//! Sequences for cc23xx_cc27xx devices
use bitfield::bitfield;
use std::ops::DerefMut;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crate::MemoryMappedRegister;
use crate::Session;
use crate::architecture::arm::ArmDebugInterface;
use crate::architecture::arm::DapAccess;
use crate::architecture::arm::DapProbe;
use crate::architecture::arm::armv6m::{Aircr, BpCtrl, Demcr, Dhcsr};
use crate::architecture::arm::core::cortex_m;
use crate::architecture::arm::dp::{Abort, Ctrl, DebugPortError, DpAccess, DpAddress, SelectV1};
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{ArmDebugSequence, cortex_m_core_start};
use crate::architecture::arm::{ArmError, FullyQualifiedApAddress};
use crate::flashing::DebugFlashSequence;
use probe_rs_target::CoreType;

/// Marker struct indicating initialization sequencing for cc23xx_cc27xx family parts.
#[derive(Debug)]
pub struct CC23xxCC27xx {
    /// Chip name - this will be used when more targets are added
    _name: String,
    /// Flag to indicate if the ROM is in the boot loop
    boot_loop: AtomicBool,
    /// Shared flag set during host-side flash programming.
    ///
    /// When true, `debug_port_start` skips the EXIT_SACI command so that the
    /// ROM SACI handler remains active for flash operations.  The flag is shared
    /// with `CC23xxCC27xxFlashSequence` via `Arc` and is reset to false when the
    /// flash sequence is dropped.
    saci_flash_mode: Arc<AtomicBool>,
}

/// Enum representing the Access Port Select register values
#[derive(Debug, Clone, Copy)]
enum ApSel {
    /// Config-AP: This is the AP used to read device type information
    CfgAp = 1,
    /// Sec-AP: This is the AP used to send SACI commands
    SecAp = 2,
}

bitfield! {
    /// Device Status Register, part of CFG-AP.
    ///
    /// This register is used to read the device status and boot status.
    #[derive(Copy, Clone)]
    pub struct DeviceStatusRegister(u32);
    impl Debug;
    ///  Bit describing if the AHB-AP is available
    ///
    /// `0`: Device is in SACI mode\
    /// `1`: Device is not in SACI mode and AHB-AP is available
    pub ahb_ap_available, _: 24;

    /// Boot Status
    ///
    /// This field is used to read the boot status of the device.
    pub u8, boot_status, _: 15, 8;
}

impl DeviceStatusRegister {
    /// Address of the device status register within the CFG-AP.
    pub const DEVICE_STATUS_REGISTER_ADDRESS: u64 = 0x0C;

    /// Read the device status register from the CFG-AP.
    pub fn read(interface: &mut dyn DapAccess) -> Result<Self, ArmError> {
        let cfg_ap: FullyQualifiedApAddress = ApSel::CfgAp.into();
        let contents =
            interface.read_raw_ap_register(&cfg_ap, Self::DEVICE_STATUS_REGISTER_ADDRESS)?;
        Ok(Self(contents))
    }
}

const BOOT_STATUS_APP_WAITLOOP_DBGPROBE: u8 = 0xC1;
const BOOT_STATUS_BLDR_WAITLOOP_DBGPROBE: u8 = 0x81;
const BOOT_STATUS_BOOT_WAITLOOP_DBGPROBE: u8 = 0x38;

bitfield! {
    /// TX_CTRL Register, part of SEC-AP.
    ///
    /// This register is used to control the transmission of SACI commands.
    #[derive(Copy, Clone)]
    pub struct TxCtrlRegister(u32);
    impl Debug;
    /// Bit indicating if the TXD register is ready.
    ///
    /// Indicates that TXD can be read. Set by hardware when TXD is written, cleared by hardware when TXD is read
    ///
    /// `0`: TXD is ready
    /// `1`: TXD is not ready
    pub txd_full, _: 0;
    /// Command Start
    ///
    /// This field is used to start a command.
    pub cmd_start, set_cmd_start: 1;
}

impl TxCtrlRegister {
    /// Address of the TX_CTRL register within the SEC-AP.
    pub const TX_CTRL_REGISTER_ADDRESS: u64 = 4;

    /// Read the TX_CTRL register from the SEC-AP.
    pub fn read(interface: &mut dyn DapAccess) -> Result<Self, ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();
        let contents = interface.read_raw_ap_register(&sec_ap, Self::TX_CTRL_REGISTER_ADDRESS)?;
        Ok(Self(contents))
    }

    /// Write the TX_CTRL register to the SEC-AP.
    pub fn write(&self, interface: &mut dyn DapAccess) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();
        interface.write_raw_ap_register(&sec_ap, Self::TX_CTRL_REGISTER_ADDRESS, self.0)
    }
}

bitfield! {
    /// RX_CTRL Register, part of SEC-AP.
    ///
    /// This register is used to control the reception of SACI responses.
    #[derive(Copy, Clone)]
    pub struct RxCtrlRegister(u32);
    impl Debug;
    /// Bit indicating if the RXD register has data ready.
    ///
    /// Indicates that RXD can be read. Set by hardware when device writes to RXD,
    /// cleared by hardware when RXD is read.
    ///
    /// `0`: RXD is empty
    /// `1`: RXD has data ready
    pub rxd_ready, _: 0;
}

impl RxCtrlRegister {
    /// Address of the RX_CTRL register within the SEC-AP.
    pub const RX_CTRL_REGISTER_ADDRESS: u64 = 0x0C;

    /// Read the RX_CTRL register from the SEC-AP.
    pub fn read(interface: &mut dyn DapAccess) -> Result<Self, ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();
        let contents = interface.read_raw_ap_register(&sec_ap, Self::RX_CTRL_REGISTER_ADDRESS)?;
        Ok(Self(contents))
    }
}

/// SACI Command IDs for flash operations.
///
/// Command IDs verified against ROM firmware (cc26xx_rom_fw/rom/fw/src/saci.c)
/// and OpenOCD reference implementation (cc_lpf3_flash.h).
#[allow(dead_code)]
mod saci_cmd {
    /// Magic key required as the second word of every flash command.
    pub const FLASH_KEY: u32 = 0xB7E3A08F;

    /// Exit SACI mode and halt at first instruction
    pub const DEBUG_EXIT_SACI_HALT: u32 = 0x07;

    /// Exit SACI mode and run the application
    pub const BLDR_APP_EXIT_SACI_RUN: u32 = 0x15;

    /// Erase entire chip (MAIN + CCFG + SCFG for CC27xx)
    pub const FLASH_ERASE_CHIP: u32 = 0x09;

    /// Program a single MAIN sector (non-pipelined, within one sector)
    pub const FLASH_PROG_MAIN_SECTOR: u32 = 0x0E;

    /// Program MAIN flash sectors using pipelined protocol
    pub const FLASH_PROG_MAIN_PIPELINED: u32 = 0x0F;

    /// Program CCFG sector (always full 512 words)
    pub const FLASH_PROG_CCFG_SECTOR: u32 = 0x0C;

    /// Verify MAIN flash sectors using CRC32
    pub const FLASH_VERIFY_MAIN_SECTORS: u32 = 0x10;

    /// Verify CCFG sector
    pub const FLASH_VERIFY_CCFG_SECTOR: u32 = 0x11;

    /// Program SCFG sector (CC27xx only)
    pub const FLASH_PROG_SCFG_SECTOR: u32 = 0x1A;

    /// Verify SCFG sector (CC27xx only)
    pub const FLASH_VERIFY_SCFG_SECTOR: u32 = 0x1B;

    /// No operation
    pub const MISC_NO_OPERATION: u32 = 0x01;
}

/// SACI command result codes.
///
/// Values verified against ROM firmware source (cc26xx_rom_fw/rom/fw/src/saci.c).
/// Success is 0x00; all error codes are in the 0x80-0xFF range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SaciResult {
    /// Command completed successfully
    Success = 0x00,
    /// Unknown or unsupported command ID
    InvalidCmdId = 0x80,
    /// Invalid address parameter
    InvalidAddressParam = 0x81,
    /// Invalid size parameter
    InvalidSizeParam = 0x82,
    /// Invalid or missing flash key (FLASH_KEY mismatch)
    InvalidKeyParam = 0x83,
    /// Flash FSM hardware error
    FlashFsmError = 0x84,
    /// Too many parameter words (buffer overflow)
    ParamBufferOverflow = 0x85,
    /// Command not allowed in current state (e.g., CCFG already programmed)
    NotAllowed = 0x86,
    /// CRC32 mismatch during verification
    Crc32Mismatch = 0x87,
    /// Blank check failed (flash not erased)
    BlankCheckFailed = 0x89,
    /// Unrecognised result code
    Unknown = 0xFF,
}

impl From<u8> for SaciResult {
    fn from(value: u8) -> Self {
        match value {
            0x00 => SaciResult::Success,
            0x80 => SaciResult::InvalidCmdId,
            0x81 => SaciResult::InvalidAddressParam,
            0x82 => SaciResult::InvalidSizeParam,
            0x83 => SaciResult::InvalidKeyParam,
            0x84 => SaciResult::FlashFsmError,
            0x85 => SaciResult::ParamBufferOverflow,
            0x86 => SaciResult::NotAllowed,
            0x87 => SaciResult::Crc32Mismatch,
            0x89 => SaciResult::BlankCheckFailed,
            _ => SaciResult::Unknown,
        }
    }
}

/// SEC-AP register addresses
#[allow(dead_code)]
mod sec_ap_regs {
    /// TX_DATA register - write command/data words here
    pub const TX_DATA: u64 = 0x00;
    /// TX_CTRL register - control transmission
    pub const TX_CTRL: u64 = 0x04;
    /// RX_DATA register - read response words here
    pub const RX_DATA: u64 = 0x08;
    /// RX_CTRL register - check for response ready
    pub const RX_CTRL: u64 = 0x0C;
}

/// Flash memory region type for CC23xx/CC27xx devices
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashRegion {
    /// Main application flash
    Main,
    /// Customer Configuration (CCFG) sector
    Ccfg,
    /// Security Configuration (SCFG) sector - CC27xx only
    Scfg,
}

impl From<ApSel> for FullyQualifiedApAddress {
    fn from(apsel: ApSel) -> Self {
        FullyQualifiedApAddress::v1_with_default_dp(apsel as u8)
    }
}

impl CC23xxCC27xx {
    /// Create the sequencer for the cc23xx_cc27xx family of parts.
    pub fn create(name: String) -> Arc<Self> {
        Arc::new(Self {
            _name: name,
            boot_loop: AtomicBool::new(false),
            saci_flash_mode: Arc::new(AtomicBool::new(false)),
        })
    }

    /// Check if the ROM is in the boot loop
    ///
    /// The boot loop is a state where the ROM is waiting for a debugger to attach and write to R3 to exit the loop.
    /// This needs to be tracked across multiple debug sequence states so it is stored on the host.
    fn is_in_boot_loop(&self) -> bool {
        self.boot_loop.load(Ordering::SeqCst)
    }

    /// Polls the TX_CTRL register until it is ready or a timeout occurs.
    ///
    /// This function reads the TX_CTRL register in a loop until it indicates readiness
    /// or the specified timeout duration has elapsed.
    ///
    /// # Arguments
    ///
    /// * `interface` - A mutable reference to the ARM communication interface.
    /// * `timeout` - The maximum duration to wait for the TX_CTRL register to be ready.
    ///
    /// # Returns
    ///
    /// * `Result<(), ArmError>` - Returns `Ok(())` if the TX_CTRL register is ready,
    ///   or an `ArmError` if there was a timeout.
    fn poll_tx_ctrl(
        &self,
        interface: &mut dyn DapAccess,
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        loop {
            let tx_ctrl = TxCtrlRegister::read(interface)?;
            if !tx_ctrl.txd_full() {
                return Ok(());
            }
            if start.elapsed() >= timeout {
                return Err(ArmError::Timeout);
            }
        }
    }

    /// Send a single-word SACI command (e.g., EXIT_SACI_HALT).
    fn saci_command(&self, interface: &mut dyn DapAccess, command: u32) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();

        self.poll_tx_ctrl(interface, Duration::from_millis(100))?;

        let mut tx_ctrl = TxCtrlRegister(0);
        tx_ctrl.set_cmd_start(true);
        TxCtrlRegister::write(&tx_ctrl, interface)?;

        interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_DATA, command)?;

        self.poll_tx_ctrl(interface, Duration::from_millis(100))?;

        Ok(())
    }
}

/// Calculate CRC32 using ISO-HDLC (CRC-32) polynomial.
///
/// Parameters from the TI documentation:
/// - CRC32_INIT  = 0xFFFFFFFF
/// - CRC32_POLY  = 0x04C11DB7
/// - CRC32_RPOLY = 0xEDB88320 (reflected)
/// - CRC32_FINAL = 0xFFFFFFFF (XOR output)
fn crc32_iso_hdlc(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;

    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }

    crc ^ 0xFFFF_FFFF
}

impl ArmDebugSequence for CC23xxCC27xx {
    fn reset_system(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        // Check if the previous code requested a halt before reset
        let demcr = Demcr(probe.read_word_32(Demcr::get_mmio_address())?);

        // Read if breakpoints should be enabled after reset
        let mut bpt_ctrl = BpCtrl(probe.read_word_32(BpCtrl::get_mmio_address())?);

        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);

        // Reset the device, flush all pending writes and wait on the reset to complete
        probe.write_word_32(Aircr::get_mmio_address(), aircr.into())?;
        probe.flush().ok();
        thread::sleep(Duration::from_millis(10));

        // Re-initializing the core(s) is on us.
        let ap = probe.fully_qualified_address();
        let interface = probe.get_arm_debug_interface()?;

        interface.reinitialize()?;
        self.debug_core_start(interface, &ap, core_type, debug_base, None)?;

        // Halt the CPU
        if demcr.vc_corereset() {
            let mut value = Dhcsr(0);
            value.set_c_halt(true);
            value.set_c_debugen(true);
            value.enable_write();

            probe.write_word_32(Dhcsr::get_mmio_address(), value.into())?;
        }

        // Restore the breakpoint control register
        bpt_ctrl.set_key(true);
        probe.write_word_32(BpCtrl::get_mmio_address(), bpt_ctrl.into())?;

        Ok(())
    }

    fn debug_port_start(
        &self,
        interface: &mut dyn DapAccess,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        // TODO:
        // Copy-pasted from the default Trait implementation, but we need to add
        // the cc23xx_cc27xx specific parts at the end
        // This code is from `debug_port_start` in `probe-rs/src/architecture/arm/sequences.rs`

        let mut abort = Abort(0);
        abort.set_dapabort(true);
        abort.set_orunerrclr(true);
        abort.set_wderrclr(true);
        abort.set_stkerrclr(true);
        abort.set_stkcmpclr(true);
        interface.write_dp_register(dp, abort)?;

        interface.write_dp_register(dp, SelectV1(0))?;

        let ctrl = interface.read_dp_register::<Ctrl>(dp)?;

        let powered_down = !(ctrl.csyspwrupack() && ctrl.cdbgpwrupack());

        if powered_down {
            tracing::info!("Debug port {dp:x?} is powered down, powering up");
            let mut ctrl = Ctrl(0);
            ctrl.set_cdbgpwrupreq(true);
            ctrl.set_csyspwrupreq(true);
            interface.write_dp_register(dp, ctrl)?;

            let start = Instant::now();
            loop {
                let ctrl = interface.read_dp_register::<Ctrl>(dp)?;
                if ctrl.csyspwrupack() && ctrl.cdbgpwrupack() {
                    break;
                }
                if start.elapsed() >= Duration::from_secs(1) {
                    return Err(ArmError::Timeout);
                }
            }

            // Init AP Transfer Mode, Transaction Counter, and Lane Mask (Normal Transfer Mode, Include all Byte Lanes)
            let mut ctrl = Ctrl(0);
            ctrl.set_cdbgpwrupreq(true);
            ctrl.set_csyspwrupreq(true);
            ctrl.set_mask_lane(0b1111);
            interface.write_dp_register(dp, ctrl)?;

            let ctrl_reg: Ctrl = interface.read_dp_register(dp)?;
            if !(ctrl_reg.csyspwrupack() && ctrl_reg.cdbgpwrupack()) {
                tracing::error!("Debug power request failed");
                return Err(DebugPortError::TargetPowerUpFailed.into());
            }

            // According to CMSIS docs, here's where we would clear errors
            // in ABORT, but we do that above instead.
        }
        // End of copy paste from `debug_port_start` in `probe-rs/src/architecture/arm/sequences.rs`

        // This code is unique to the cc23xx_cc27xx family
        // First connect to the config AP to read the device status register
        // This will tell us the state of the boot rom and if SACI is enabled

        // Read the device status register
        let mut device_status = DeviceStatusRegister::read(interface)?;

        // AHB-AP is not accessible when in SACI mode.
        if !device_status.ahb_ap_available() {
            if self.saci_flash_mode.load(Ordering::SeqCst) {
                // Flash programming is in progress — stay in SACI mode so the
                // ROM flash handler remains active for subsequent SACI commands.
                tracing::info!(
                    "CC23xx/CC27xx: debug_port_start in flash mode, leaving SACI active"
                );
                return Ok(());
            }

            // Normal debug session — exit SACI so the AHB-AP becomes accessible.
            // debug_port_connect already asserted nRESET so the ROM starts fresh
            // with isExitAllowed=true, meaning EXIT_SACI_HALT should succeed.
            self.saci_command(interface, saci_cmd::DEBUG_EXIT_SACI_HALT)?;

            thread::sleep(Duration::from_millis(30));
            device_status = DeviceStatusRegister::read(interface)?;

            // Check if the boot ROM is waiting for a debugger to attach.
            match device_status.boot_status() {
                BOOT_STATUS_BOOT_WAITLOOP_DBGPROBE
                | BOOT_STATUS_BLDR_WAITLOOP_DBGPROBE
                | BOOT_STATUS_APP_WAITLOOP_DBGPROBE => {
                    tracing::info!("BOOT_WAITLOOP_DBGPROBE");
                    self.boot_loop.store(true, Ordering::SeqCst);
                }
                _ => {
                    if !device_status.ahb_ap_available() {
                        tracing::warn!(
                            "CC23xx/CC27xx: Device is still in SACI mode after EXIT_SACI_HALT"
                        );
                    }
                }
            }
        }

        Ok(())
    }

    fn debug_core_start(
        &self,
        interface: &mut dyn ArmDebugInterface,
        core_ap: &FullyQualifiedApAddress,
        _core_type: CoreType,
        _debug_base: Option<u64>,
        _cti_base: Option<u64>,
    ) -> Result<(), ArmError> {
        if self.is_in_boot_loop() {
            // Step 1: Halt the CPU
            let mut dhcsr = Dhcsr(0);
            dhcsr.set_c_halt(true);
            dhcsr.set_c_debugen(true);
            dhcsr.enable_write();

            let mut memory = interface.memory_interface(core_ap)?;
            memory.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;

            // Step 1.1: Wait for the CPU to halt
            dhcsr = Dhcsr(memory.read_word_32(Dhcsr::get_mmio_address())?);
            while !dhcsr.s_halt() {
                dhcsr = Dhcsr(memory.read_word_32(Dhcsr::get_mmio_address())?);
            }

            // Step 2: Write R3 to 0 to exit the boot loop
            cortex_m::write_core_reg(memory.deref_mut(), crate::RegisterId(3), 0x00000000)?;

            // Step 3: Clear the BOOT_LOOP flag
            self.boot_loop.store(false, Ordering::SeqCst);
        }

        // Step 4: Start the core like normal
        let mut core = interface.memory_interface(core_ap)?;
        cortex_m_core_start(&mut *core)
    }

    fn debug_flash_sequence(&self) -> Option<Arc<dyn DebugFlashSequence>> {
        Some(Arc::new(CC23xxCC27xxFlashSequence::new_with_flag(
            Arc::clone(&self.saci_flash_mode),
        )))
    }

    /// Assert and deassert nRESET on every connect.
    ///
    /// This matches OpenOCD's behaviour for CC27xx: every `init` begins with
    /// `adapter assert srst` so the device always starts from a known state
    /// (ROM SACI handler active, isExitAllowed=true). Without this, if the
    /// device was left stuck in SACI mode with isExitAllowed=false (e.g., after
    /// CCFG/SCFG programming), it would be impossible to exit SACI via software
    /// commands alone.
    ///
    /// After the reset the default SWD reconnect sequence runs normally, and
    /// `debug_port_start` then exits SACI (or stays in it during flash mode).
    fn debug_port_connect(
        &self,
        interface: &mut dyn DapProbe,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        tracing::info!("CC23xx/CC27xx: Asserting nRESET before SWD connect (OpenOCD-compatible)");

        // Assert nRESET (drive low).
        self.reset_hardware_assert(interface)?;
        thread::sleep(Duration::from_millis(5));

        // Deassert nRESET (release/drive high): set both pin_output and pin_select
        // bits for nRESET so the probe drives nRESET high.
        let mut n_reset = crate::architecture::arm::traits::Pins(0);
        n_reset.set_nreset(true);
        let _ = interface.swj_pins(n_reset.0 as u32, n_reset.0 as u32, 0)?;

        // Give the ROM time to reach the SACI handler before the SWD connect
        // sequence reads DPIDR (60 ms matches OpenOCD's timing).
        thread::sleep(Duration::from_millis(60));

        // Delegate to the default SWD connect sequence.
        let default = crate::architecture::arm::sequences::DefaultArmSequence::create();
        default.debug_port_connect(interface, dp)
    }
}

/// Host-side flash programming implementation for CC23xx/CC27xx devices.
///
/// This implements flash programming via SACI commands sent through the SEC-AP
/// rather than loading a flash algorithm into target RAM.
///
/// The `saci_flash_mode` flag is shared with `CC23xxCC27xx`.  While this struct
/// is alive, the flag is `true`, which tells `debug_port_start` to skip the
/// EXIT_SACI command so the ROM SACI handler stays active.  The flag is
/// automatically reset to `false` when this struct is dropped.
#[derive(Debug)]
pub struct CC23xxCC27xxFlashSequence {
    /// Shared flag with CC23xxCC27xx to suppress EXIT_SACI during flash.
    saci_flash_mode: Arc<AtomicBool>,
}

impl CC23xxCC27xxFlashSequence {
    /// Create a flash sequence sharing the given saci_flash_mode flag.
    ///
    /// The flag is set to `true` immediately so that the next `debug_port_start`
    /// call (triggered by `reinitialize()` in `reset_into_saci_mode`) skips the
    /// EXIT_SACI command.
    pub fn new_with_flag(saci_flash_mode: Arc<AtomicBool>) -> Self {
        saci_flash_mode.store(true, Ordering::SeqCst);
        Self { saci_flash_mode }
    }

    /// Fallback constructor (no shared flag — used in tests or standalone contexts).
    pub fn new() -> Self {
        Self::new_with_flag(Arc::new(AtomicBool::new(false)))
    }

    /// Build the first parameter word (header) for a SACI command.
    ///
    /// Format per ROM source: bits[7:0]=cmd_id, bits[15:8]=resp_seq_num, bits[31:16]=cmd_specific.
    fn make_header(cmd_id: u32, cmd_specific: u32) -> u32 {
        (cmd_specific << 16) | cmd_id
    }

    /// Poll TX_CTRL until TXD_FULL clears or timeout.
    fn poll_tx_ctrl(
        &self,
        interface: &mut dyn DapAccess,
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        loop {
            let tx_ctrl = TxCtrlRegister::read(interface)?;
            if !tx_ctrl.txd_full() {
                return Ok(());
            }
            if start.elapsed() >= timeout {
                return Err(ArmError::Timeout);
            }
        }
    }

    /// Poll RX_CTRL until RXD_READY is set or timeout.
    fn poll_rx_ctrl(
        &self,
        interface: &mut dyn DapAccess,
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        loop {
            let rx_ctrl = RxCtrlRegister::read(interface)?;
            if rx_ctrl.rxd_ready() {
                return Ok(());
            }
            if start.elapsed() >= timeout {
                return Err(ArmError::Timeout);
            }
            thread::sleep(Duration::from_micros(5));
        }
    }

    /// Send a sequence of SACI command words.
    ///
    /// Follows the OpenOCD `cc_lpf3_saci_send_cmd` protocol:
    /// 1. Poll TXD_FULL clear
    /// 2. Set CMD_START in TXCTL, write word[0]
    /// 3. Clear CMD_START
    /// 4. For each subsequent word: poll TXD_FULL clear, write word
    /// 5. Final poll TXD_FULL clear
    fn saci_send_words(
        &self,
        interface: &mut dyn DapAccess,
        words: &[u32],
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();

        if words.is_empty() {
            return Ok(());
        }

        // Poll until TX buffer is ready for a new command.
        self.poll_tx_ctrl(interface, timeout)?;

        // Set CMD_START to mark the beginning of a new command, then write the first word.
        let mut tx_ctrl = TxCtrlRegister(0);
        tx_ctrl.set_cmd_start(true);
        TxCtrlRegister::write(&tx_ctrl, interface)?;
        interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_DATA, words[0])?;

        if words.len() > 1 {
            // Wait for first word to be consumed, then clear CMD_START before
            // sending continuation words.
            self.poll_tx_ctrl(interface, timeout)?;
            interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_CTRL, 0)?;

            for word in &words[1..] {
                self.poll_tx_ctrl(interface, timeout)?;
                interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_DATA, *word)?;
            }
        }

        // Final poll to confirm the last word has been consumed.
        self.poll_tx_ctrl(interface, timeout)?;

        Ok(())
    }

    /// Read one response word from the device.
    fn saci_read_response(
        &self,
        interface: &mut dyn DapAccess,
        timeout: Duration,
    ) -> Result<u32, ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();
        self.poll_rx_ctrl(interface, timeout)?;
        let response = interface.read_raw_ap_register(&sec_ap, sec_ap_regs::RX_DATA)?;
        Ok(response)
    }

    /// Check a SACI response word and return an error if the result is not Success.
    fn check_saci_result(response: u32, context: &str) -> Result<(), ArmError> {
        let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
        if result != SaciResult::Success {
            tracing::error!(
                "SACI {} failed: {:?} (raw response: 0x{:08X})",
                context,
                result,
                response
            );
            return Err(ArmError::Other(format!(
                "SACI {} failed: {:?}",
                context, result
            )));
        }
        Ok(())
    }

    /// Pack byte slice into little-endian u32 words, padding the last word with `pad`.
    fn pack_words(data: &[u8], pad: u8) -> Vec<u32> {
        data.chunks(4)
            .map(|chunk| {
                let mut word = 0u32;
                for (i, &byte) in chunk.iter().enumerate() {
                    word |= (byte as u32) << (i * 8);
                }
                for i in chunk.len()..4 {
                    word |= (pad as u32) << (i * 8);
                }
                word
            })
            .collect()
    }

    /// Program a sector of MAIN flash using FLASH_PROG_MAIN_SECTOR (0x0E).
    fn program_main(
        &self,
        interface: &mut dyn DapAccess,
        address: u64,
        data: &[u8],
    ) -> Result<(), ArmError> {
        let byte_count = data.len() as u32;
        let header = Self::make_header(saci_cmd::FLASH_PROG_MAIN_SECTOR, byte_count);
        let addr_word = address as u32;

        let mut words = vec![header, saci_cmd::FLASH_KEY, addr_word];
        words.extend(Self::pack_words(data, 0xFF));

        self.saci_send_words(interface, &words, Duration::from_millis(200))?;
        let response = self.saci_read_response(interface, Duration::from_secs(5))?;
        Self::check_saci_result(
            response,
            &format!("FLASH_PROG_MAIN_SECTOR at 0x{address:08X}"),
        )?;
        Ok(())
    }

    /// Program the CCFG sector using FLASH_PROG_CCFG_SECTOR (0x0C).
    ///
    /// Pads data to exactly 512 words (2048 bytes) with 0xFF.  Sets skip_user_rec=1.
    fn program_ccfg(&self, interface: &mut dyn DapAccess, data: &[u8]) -> Result<(), ArmError> {
        // skip_user_rec = bit 0 of cmd_specific
        let header = Self::make_header(saci_cmd::FLASH_PROG_CCFG_SECTOR, 0x0001);

        // Pad to exactly 2048 bytes / 512 words.
        let mut padded = vec![0xFFu8; 2048];
        let copy_len = data.len().min(2048);
        padded[..copy_len].copy_from_slice(&data[..copy_len]);

        let mut words = vec![header, saci_cmd::FLASH_KEY];
        words.extend(Self::pack_words(&padded, 0xFF));

        self.saci_send_words(interface, &words, Duration::from_millis(200))?;
        let response = self.saci_read_response(interface, Duration::from_secs(5))?;
        Self::check_saci_result(response, "FLASH_PROG_CCFG_SECTOR")?;
        Ok(())
    }

    /// Program the SCFG sector using FLASH_PROG_SCFG_SECTOR (0x1A).
    ///
    /// byte_count is the number of bytes to program (encoded in cmd_specific).
    fn program_scfg(&self, interface: &mut dyn DapAccess, data: &[u8]) -> Result<(), ArmError> {
        let byte_count = data.len() as u32;
        let header = Self::make_header(saci_cmd::FLASH_PROG_SCFG_SECTOR, byte_count);

        let mut words = vec![header, saci_cmd::FLASH_KEY];
        words.extend(Self::pack_words(data, 0xFF));

        self.saci_send_words(interface, &words, Duration::from_millis(200))?;
        let response = self.saci_read_response(interface, Duration::from_secs(5))?;
        Self::check_saci_result(response, "FLASH_PROG_SCFG_SECTOR")?;
        Ok(())
    }
}

impl CC23xxCC27xxFlashSequence {
    /// Reset the device via hardware nRESET (SRST) and wait for SACI mode.
    ///
    /// `debug_port_start` exits SACI mode so that the AHB-AP becomes accessible
    /// for normal debugging.  Before flash commands can be sent, the device must
    /// be reset back into SACI mode via a hardware reset of the nRESET pin.
    ///
    /// SYSRESETRQ (writing AIRCR) does **not** trigger the ROM to re-enter SACI;
    /// only a hardware reset does.  This matches the OpenOCD sequence in
    /// `ti_cc27xx.cfg`:
    ///   adapter assert srst → 5 ms → deassert → 60 ms → dap init → 100 ms
    fn reset_into_saci_mode(&self, interface: &mut dyn ArmDebugInterface) -> Result<(), ArmError> {
        tracing::info!("CC23xx/CC27xx: Re-initializing to enter SACI mode for flash programming");

        // `reinitialize()` calls `debug_port_connect` which now always asserts
        // nRESET, causing the ROM to re-enter SACI mode.  `debug_port_start` will
        // see `saci_flash_mode == true` (set by new_with_flag) and skip EXIT_SACI.
        interface.reinitialize()?;

        // Give the ROM time to reach the SACI handler after reinitialise.
        thread::sleep(Duration::from_millis(100));

        // Confirm SACI mode by checking ahb_ap_available == false.
        let start = Instant::now();
        loop {
            if matches!(DeviceStatusRegister::read(interface), Ok(s) if !s.ahb_ap_available()) {
                tracing::info!("CC23xx/CC27xx: Device is in SACI mode, ready for flash");
                return Ok(());
            }
            if start.elapsed() >= Duration::from_secs(3) {
                tracing::error!(
                    "CC23xx/CC27xx: Timeout waiting for SACI mode. \
                     Ensure nRESET is connected to the debug probe."
                );
                return Err(ArmError::Timeout);
            }
            thread::sleep(Duration::from_millis(50));
        }
    }
}

impl Drop for CC23xxCC27xxFlashSequence {
    fn drop(&mut self) {
        // Reset the flash mode flag so subsequent debug sessions exit SACI normally.
        self.saci_flash_mode.store(false, Ordering::SeqCst);
    }
}

impl Default for CC23xxCC27xxFlashSequence {
    fn default() -> Self {
        Self::new()
    }
}

/// Address boundary constants for CCFG and SCFG regions on CC23xx/CC27xx devices.
///
/// Addresses derived from the YAML memory map for CC23XX_CC27XX_Series.
const CCFG_START: u64 = 0x4E02_0000;
const SCFG_START: u64 = 0x4E04_0000;

impl DebugFlashSequence for CC23xxCC27xxFlashSequence {
    fn erase_all(&self, session: &mut Session) -> Result<(), crate::Error> {
        let interface = session.get_arm_interface()?;

        // debug_port_start exited SACI mode to access the AHB-AP.  A hardware
        // reset via nRESET is required to re-enter SACI mode before flash
        // commands can be accepted.
        self.reset_into_saci_mode(interface)?;

        tracing::info!("CC23xx/CC27xx: Chip erase via SACI FLASH_ERASE_CHIP (0x09)");

        // FLASH_ERASE_CHIP (0x09): [header, FLASH_KEY]
        let header = Self::make_header(saci_cmd::FLASH_ERASE_CHIP, 0);
        let words = [header, saci_cmd::FLASH_KEY];
        self.saci_send_words(interface, &words, Duration::from_millis(200))?;

        let response = self.saci_read_response(interface, Duration::from_secs(30))?;
        Self::check_saci_result(response, "FLASH_ERASE_CHIP")?;

        tracing::info!("CC23xx/CC27xx: Chip erase completed");
        Ok(())
    }

    fn program(
        &self,
        session: &mut Session,
        address: u64,
        data: &[u8],
    ) -> Result<(), crate::Error> {
        tracing::debug!(
            "CC23xx/CC27xx: Programming {} bytes at 0x{:08X}",
            data.len(),
            address
        );

        let interface = session.get_arm_interface()?;

        if address >= SCFG_START {
            self.program_scfg(interface, data)?;
        } else if address >= CCFG_START {
            self.program_ccfg(interface, data)?;
        } else {
            self.program_main(interface, address, data)?;
        }
        Ok(())
    }

    fn verify(
        &self,
        session: &mut Session,
        address: u64,
        data: &[u8],
    ) -> Result<bool, crate::Error> {
        tracing::debug!(
            "CC23xx/CC27xx: Verifying {} bytes at 0x{:08X}",
            data.len(),
            address
        );

        let interface = session.get_arm_interface()?;

        if address >= SCFG_START {
            // FLASH_VERIFY_SCFG_SECTOR (0x1B): [header(check_exp_crc=1), expCrc32]
            //
            // The ROM verifies CRC over only the first 0xE4 (228) bytes of SCFG,
            // matching the range covered by Scfg::update_crcs() and OpenOCD's
            // SCFG_CONTENT_SIZE constant.  The trailing key-ring slots are excluded.
            const SCFG_CRC_BYTE_COUNT: usize = 0xE4;
            let crc_data = &data[..data.len().min(SCFG_CRC_BYTE_COUNT)];
            let expected_crc = crc32_iso_hdlc(crc_data);
            let header = Self::make_header(saci_cmd::FLASH_VERIFY_SCFG_SECTOR, 0x0001);
            let words = [header, expected_crc];
            self.saci_send_words(interface, &words, Duration::from_millis(100))?;
            let response = self.saci_read_response(interface, Duration::from_secs(10))?;
            let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
            match result {
                SaciResult::Success => Ok(true),
                SaciResult::Crc32Mismatch => Ok(false),
                _ => Err(ArmError::Other(format!(
                    "SACI FLASH_VERIFY_SCFG_SECTOR failed: {result:?}"
                ))
                .into()),
            }
        } else if address >= CCFG_START {
            // FLASH_VERIFY_CCFG_SECTOR (0x11): simplest form — check_exp_crcs=0,
            // skip_user_rec=1 — lets the ROM verify the CRCs embedded in the CCFG
            // rather than requiring the host to supply external CRCs.
            let header = Self::make_header(saci_cmd::FLASH_VERIFY_CCFG_SECTOR, 0x0002);
            let words = [header, 0u32, 0u32, 0u32, 0u32];
            self.saci_send_words(interface, &words, Duration::from_millis(100))?;
            let response = self.saci_read_response(interface, Duration::from_secs(10))?;
            let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
            match result {
                SaciResult::Success => Ok(true),
                SaciResult::Crc32Mismatch | SaciResult::BlankCheckFailed => Ok(false),
                _ => Err(ArmError::Other(format!(
                    "SACI FLASH_VERIFY_CCFG_SECTOR failed: {result:?}"
                ))
                .into()),
            }
        } else {
            // FLASH_VERIFY_MAIN_SECTORS (0x10): [header, firstSectorAddr, byteCount, expCrc32]
            let expected_crc = crc32_iso_hdlc(data);
            let header = Self::make_header(saci_cmd::FLASH_VERIFY_MAIN_SECTORS, 0);
            let words = [header, address as u32, data.len() as u32, expected_crc];
            self.saci_send_words(interface, &words, Duration::from_millis(100))?;
            let response = self.saci_read_response(interface, Duration::from_secs(10))?;
            let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
            match result {
                SaciResult::Success => Ok(true),
                SaciResult::Crc32Mismatch => Ok(false),
                _ => Err(ArmError::Other(format!(
                    "SACI FLASH_VERIFY_MAIN_SECTORS failed: {result:?}"
                ))
                .into()),
            }
        }
    }

    fn supports_sector_erase(&self) -> bool {
        false
    }

    fn prepare_verify(&self, session: &mut Session) -> Result<(), crate::Error> {
        let interface = session.get_arm_interface()?;

        // When verify runs as a separate pass after finish_flash() exited SACI,
        // the device is in normal debug mode.  We need to re-enter SACI to send
        // FLASH_VERIFY_* commands.  If already in SACI mode (inline verify during
        // program()), this is a no-op.
        if matches!(DeviceStatusRegister::read(interface), Ok(s) if s.ahb_ap_available()) {
            tracing::info!("CC23xx/CC27xx: Prepare verify — device not in SACI, re-entering");
            self.reset_into_saci_mode(interface)?;
        }
        Ok(())
    }

    fn finish_flash(&self, session: &mut Session) -> Result<(), crate::Error> {
        // The session is reused between the flash operation and post-flash debug
        // access, so the device must be out of SACI mode before we return.
        //
        // Sequence (matches OpenOCD `cc27xx reset_halt` after programming):
        // 1. Clear saci_flash_mode so the next debug_port_start exits SACI.
        // 2. Call reinitialize() which triggers debug_port_connect (nRESET → SACI)
        //    then debug_port_start (EXIT_SACI_HALT → device boots, AHB-AP available).
        tracing::info!("CC23xx/CC27xx: Flash complete, exiting SACI for normal debug access");

        // Step 1: Clear the flag BEFORE reinitialize so debug_port_start sends
        // EXIT_SACI_HALT instead of staying in SACI mode.
        self.saci_flash_mode.store(false, Ordering::SeqCst);

        // Step 2: Reinitialize → debug_port_connect (nRESET) → debug_port_start
        // (EXIT_SACI_HALT) → device exits SACI and enters boot wait loop.
        let interface = session.get_arm_interface()?;
        interface.reinitialize()?;

        Ok(())
    }
}
