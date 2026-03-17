//! Sequences for cc23xx_cc27xx devices
use bitfield::bitfield;
use std::ops::DerefMut;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crate::MemoryMappedRegister;
use crate::architecture::arm::ArmDebugInterface;
use crate::architecture::arm::DapAccess;
use crate::architecture::arm::armv6m::{Aircr, BpCtrl, Demcr, Dhcsr};
use crate::architecture::arm::core::cortex_m;
use crate::architecture::arm::dp::{Abort, Ctrl, DebugPortError, DpAccess, DpAddress, SelectV1};
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{ArmDebugSequence, DebugFlashSequence, cortex_m_core_start};
use crate::architecture::arm::{ArmError, FullyQualifiedApAddress};
use probe_rs_target::{CoreType, FlashProperties, SectorDescription};

/// Marker struct indicating initialization sequencing for cc23xx_cc27xx family parts.
#[derive(Debug)]
pub struct CC23xxCC27xx {
    /// Chip name - this will be used when more targets are added
    _name: String,
    /// Flag to indicate if the ROM is in the boot loop
    boot_loop: AtomicBool,
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
/// These command IDs are defined in the CC23xx/CC27xx Technical Reference Manual.
#[allow(dead_code)]
mod saci_cmd {
    /// Exit SACI mode
    pub const DEBUG_EXIT_SACI: u32 = 0x07;

    /// Erase entire chip (MAIN + CCFG, SCFG, OTP for CC27xx)
    pub const FLASH_ERASE_CHIP: u32 = 0x05;

    /// Program MAIN flash sectors using pipelined protocol
    pub const FLASH_PROG_MAIN_PIPELINED: u32 = 0x06;

    /// Program CCFG sector
    pub const FLASH_PROG_CCFG_SECTOR: u32 = 0x10;

    /// Verify MAIN flash sectors using CRC32
    pub const FLASH_VERIFY_MAIN_SECTORS: u32 = 0x11;

    /// Verify CCFG sector using CRC32
    pub const FLASH_VERIFY_CCFG_SECTOR: u32 = 0x12;

    /// Program SCFG sector (CC27xx only)
    pub const FLASH_PROG_SCFG_SECTOR: u32 = 0x1A;

    /// Verify SCFG sector (CC27xx only)
    pub const FLASH_VERIFY_SCFG_SECTOR: u32 = 0x1B;

    /// No operation - used to keep SACI session alive
    pub const MISC_NO_OPERATION: u32 = 0x00;
}

/// SACI command result codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SaciResult {
    /// Command completed successfully
    Success = 0x00,
    /// Invalid command parameter
    InvalidParam = 0x01,
    /// Flash FSM error
    FlashFsmError = 0x02,
    /// Command not allowed in current state
    NotAllowed = 0x03,
    /// CRC mismatch during verification
    CrcMismatch = 0x04,
    /// Unknown error
    Unknown = 0xFF,
}

impl From<u8> for SaciResult {
    fn from(value: u8) -> Self {
        match value {
            0x00 => SaciResult::Success,
            0x01 => SaciResult::InvalidParam,
            0x02 => SaciResult::FlashFsmError,
            0x03 => SaciResult::NotAllowed,
            0x04 => SaciResult::CrcMismatch,
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
        let mut tx_ctrl = TxCtrlRegister::read(interface)?;
        TxCtrlRegister::read(interface)?;
        while tx_ctrl.txd_full() {
            if start.elapsed() >= timeout {
                return Err(ArmError::Timeout);
            }
            tx_ctrl = TxCtrlRegister::read(interface)?;
        }
        Ok(())
    }

    /// Sends a SACI command to the device.
    ///
    /// This function communicates with the device using the Security Access Port (SEC AP)
    /// to send a SACI command. It waits for the TX_CTRL register to be ready before sending
    /// the command and then writes the command to the TX_DATA register. Again waiting for TX_CTRL to be ready.
    ///
    /// Implements Section 8.3.1.1 from https://www.ti.com/lit/ug/swcu193/swcu193.pdf
    ///
    /// # Arguments
    ///
    /// * `interface` - A mutable reference to the ARM communication interface.
    /// * `command` - The SACI command to be sent.
    ///
    /// # Returns
    ///
    /// * `Result<(), ArmError>` - Returns `Ok(())` if the command was successfully sent,
    ///   or an `ArmError` if there was an error during communication.
    ///
    fn saci_command(&self, interface: &mut dyn DapAccess, command: u32) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();

        const TX_DATA_ADDR: u64 = 0;

        // Wait for tx_ctrl to be ready with a timeout of 1 millisecond
        self.poll_tx_ctrl(interface, Duration::from_millis(1))?;

        // Set Cmd Start
        let mut tx_ctrl = TxCtrlRegister(0);
        tx_ctrl.set_cmd_start(true);
        TxCtrlRegister::write(&tx_ctrl, interface)?;

        // Write parameter word to txd
        interface.write_raw_ap_register(&sec_ap, TX_DATA_ADDR, command)?;

        self.poll_tx_ctrl(interface, Duration::from_millis(1))?;

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

        // AHB-AP is not accessible when in SACI mode, so exit SACI
        if !device_status.ahb_ap_available() {
            // Send the SACI command to exit SACI
            self.saci_command(interface, 0x07)?;

            // Read the device status register again to check if boot is completed
            device_status = DeviceStatusRegister::read(interface)?;

            // Check if the boot rom is waiting for a debugger to attach
            match device_status.boot_status() {
                BOOT_STATUS_BOOT_WAITLOOP_DBGPROBE
                | BOOT_STATUS_BLDR_WAITLOOP_DBGPROBE
                | BOOT_STATUS_APP_WAITLOOP_DBGPROBE => {
                    tracing::info!("BOOT_WAITLOOP_DBGPROBE");
                    self.boot_loop.store(true, Ordering::SeqCst);
                }
                _ => tracing::warn!("Expected device to be waiting on debugger, but it is not"),
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
        Some(Arc::new(CC23xxCC27xxFlashSequence::new()))
    }
}

/// Host-side flash programming implementation for CC23xx/CC27xx devices.
///
/// This implements flash programming via SACI commands sent through the SEC-AP
/// rather than loading a flash algorithm into target RAM.
#[derive(Debug)]
pub struct CC23xxCC27xxFlashSequence {
    /// Flash properties for the device
    properties: FlashProperties,
}

impl CC23xxCC27xxFlashSequence {
    /// Create a new flash sequence for CC23xx/CC27xx devices.
    pub fn new() -> Self {
        Self {
            properties: FlashProperties {
                address_range: 0..0x0008_0000, // 512KB default, adjusted per device
                page_size: 2048,               // 2KB page size
                erased_byte_value: 0xFF,
                program_page_timeout: 1000,    // 1 second
                erase_sector_timeout: 5000,    // 5 seconds
                sectors: vec![SectorDescription {
                    size: 2048,
                    address: 0,
                }],
            },
        }
    }

    /// Helper to send SACI commands (delegates to static helper).
    fn poll_tx_ctrl(
        &self,
        interface: &mut dyn DapAccess,
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        let mut tx_ctrl = TxCtrlRegister::read(interface)?;
        TxCtrlRegister::read(interface)?;
        while tx_ctrl.txd_full() {
            if start.elapsed() >= timeout {
                return Err(ArmError::Timeout);
            }
            tx_ctrl = TxCtrlRegister::read(interface)?;
        }
        Ok(())
    }

    /// Helper to poll RX_CTRL until data is ready.
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
            thread::sleep(Duration::from_micros(100));
        }
    }

    /// Send a simple SACI command.
    fn saci_command(&self, interface: &mut dyn DapAccess, command: u32) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();

        self.poll_tx_ctrl(interface, Duration::from_millis(1))?;

        let mut tx_ctrl = TxCtrlRegister(0);
        tx_ctrl.set_cmd_start(true);
        TxCtrlRegister::write(&tx_ctrl, interface)?;

        interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_DATA, command)?;

        self.poll_tx_ctrl(interface, Duration::from_millis(1))?;

        Ok(())
    }

    /// Send a multi-word SACI command.
    fn saci_command_multi(
        &self,
        interface: &mut dyn DapAccess,
        words: &[u32],
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::SecAp.into();

        if words.is_empty() {
            return Ok(());
        }

        self.poll_tx_ctrl(interface, timeout)?;

        let mut tx_ctrl = TxCtrlRegister(0);
        tx_ctrl.set_cmd_start(true);
        TxCtrlRegister::write(&tx_ctrl, interface)?;

        interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_DATA, words[0])?;

        for word in &words[1..] {
            self.poll_tx_ctrl(interface, timeout)?;
            interface.write_raw_ap_register(&sec_ap, sec_ap_regs::TX_DATA, *word)?;
        }

        self.poll_tx_ctrl(interface, timeout)?;

        Ok(())
    }

    /// Read a response word from the device.
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
}

impl Default for CC23xxCC27xxFlashSequence {
    fn default() -> Self {
        Self::new()
    }
}

impl DebugFlashSequence for CC23xxCC27xxFlashSequence {
    fn erase_all(&self, interface: &mut dyn ArmDebugInterface) -> Result<(), ArmError> {
        tracing::info!("CC23xx/CC27xx: Erasing all flash via SACI");

        // Send erase chip command
        self.saci_command(interface, saci_cmd::FLASH_ERASE_CHIP)?;

        // Wait for erase to complete (can take several seconds)
        let response = self.saci_read_response(interface, Duration::from_secs(30))?;

        // Check result (bits 23:16 of response)
        let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
        if result != SaciResult::Success {
            tracing::error!("Flash erase chip failed with result: {:?}", result);
            return Err(ArmError::Timeout);
        }

        tracing::info!("Flash erase chip completed successfully");
        Ok(())
    }

    fn erase_sector(
        &self,
        _interface: &mut dyn ArmDebugInterface,
        address: u64,
    ) -> Result<(), ArmError> {
        // CC23xx/CC27xx doesn't support individual sector erase
        // The only way to erase is full chip erase
        tracing::warn!(
            "CC23xx/CC27xx: Individual sector erase at 0x{:08X} not supported, requires full chip erase",
            address
        );
        Err(ArmError::NotImplemented("sector erase - use erase_all"))
    }

    fn program(
        &self,
        interface: &mut dyn ArmDebugInterface,
        address: u64,
        data: &[u8],
    ) -> Result<(), ArmError> {
        tracing::debug!(
            "CC23xx/CC27xx: Programming {} bytes at 0x{:08X}",
            data.len(),
            address
        );

        // Build command header
        let word_count = data.len().div_ceil(4);
        let cmd_word = saci_cmd::FLASH_PROG_MAIN_PIPELINED | ((word_count as u32) << 16);
        let addr_word = address as u32;

        let mut words = vec![cmd_word, addr_word];

        // Add data words (little endian, pad to word boundary)
        for chunk in data.chunks(4) {
            let mut word = 0u32;
            for (i, &byte) in chunk.iter().enumerate() {
                word |= (byte as u32) << (i * 8);
            }
            // Pad with 0xFF if chunk is less than 4 bytes
            for i in chunk.len()..4 {
                word |= 0xFF << (i * 8);
            }
            words.push(word);
        }

        // Send all words
        self.saci_command_multi(interface, &words, Duration::from_millis(100))?;

        // Wait for programming to complete
        let response = self.saci_read_response(interface, Duration::from_secs(5))?;

        // Check result
        let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
        if result != SaciResult::Success {
            tracing::error!(
                "Flash program at 0x{:08X} failed with result: {:?}",
                address,
                result
            );
            return Err(ArmError::Timeout);
        }

        Ok(())
    }

    fn verify(
        &self,
        interface: &mut dyn ArmDebugInterface,
        address: u64,
        data: &[u8],
    ) -> Result<bool, ArmError> {
        tracing::debug!(
            "CC23xx/CC27xx: Verifying {} bytes at 0x{:08X}",
            data.len(),
            address
        );

        // Calculate expected CRC32
        let expected_crc = crc32_iso_hdlc(data);

        // Build command
        let word_count = data.len().div_ceil(4);
        let cmd_word = saci_cmd::FLASH_VERIFY_MAIN_SECTORS | ((word_count as u32) << 16);
        let addr_word = address as u32;
        let crc_word = expected_crc;

        let words = [cmd_word, addr_word, crc_word];
        self.saci_command_multi(interface, &words, Duration::from_millis(10))?;

        // Wait for verification
        let response = self.saci_read_response(interface, Duration::from_secs(10))?;

        // Check result
        let result = SaciResult::from(((response >> 16) & 0xFF) as u8);
        match result {
            SaciResult::Success => Ok(true),
            SaciResult::CrcMismatch => Ok(false),
            _ => {
                tracing::error!("Flash verify failed with result: {:?}", result);
                Err(ArmError::Timeout)
            }
        }
    }

    fn flash_properties(&self) -> &FlashProperties {
        &self.properties
    }
}
