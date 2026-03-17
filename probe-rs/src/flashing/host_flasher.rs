//! Host-side flash programming implementation.
//!
//! This module provides flash programming via the host PC using debug interface
//! commands (e.g., SACI for TI CC23xx/CC27xx), rather than loading a flash
//! algorithm into target RAM.

use std::sync::Arc;
use std::time::Instant;

use probe_rs_target::{NvmRegion, RawFlashAlgorithm};

use super::builder::FlashBuilder;
use super::{FlashError, FlashLayout, FlashProgress};
use crate::architecture::arm::sequences::DebugFlashSequence;
use crate::flashing::flasher::FlashData;
use crate::session::Session;

/// A region loaded for host-side flash programming.
pub(super) struct HostLoadedRegion {
    /// The memory region being programmed.
    #[allow(dead_code)] // May be used for error reporting or debugging
    pub region: NvmRegion,
    /// The flash data to program.
    pub data: FlashData,
}

impl HostLoadedRegion {
    /// Returns the flash layout for this region.
    pub fn flash_layout(&self) -> &FlashLayout {
        self.data.layout()
    }
}

/// A flasher that uses host-side programming via debug interface commands.
///
/// Instead of loading a flash algorithm into target RAM and executing it,
/// this flasher sends commands directly to the device via the debug interface.
/// This is used for devices like TI CC23xx/CC27xx that support SACI commands
/// for flash programming.
pub struct HostSideFlasher {
    /// The debug flash sequence implementation.
    flash_sequence: Arc<dyn DebugFlashSequence>,
    /// The core index to use.
    pub(super) core_index: usize,
    /// The raw flash algorithm (for metadata like name).
    pub(super) flash_algorithm: RawFlashAlgorithm,
    /// Regions to program.
    pub(super) regions: Vec<HostLoadedRegion>,
}

impl HostSideFlasher {
    /// Create a new host-side flasher.
    pub fn new(
        flash_sequence: Arc<dyn DebugFlashSequence>,
        core_index: usize,
        raw_flash_algorithm: RawFlashAlgorithm,
    ) -> Self {
        Self {
            flash_sequence,
            core_index,
            flash_algorithm: raw_flash_algorithm,
            regions: Vec::new(),
        }
    }

    /// Add a region to be programmed.
    pub(crate) fn add_region(
        &mut self,
        region: NvmRegion,
        builder: &FlashBuilder,
        restore_unwritten_bytes: bool,
    ) -> Result<(), FlashError> {
        // Build flash layout for this region using the flash properties from the sequence
        let flash_props = self.flash_sequence.flash_properties();
        let layout = builder.build_sectors_and_pages_from_properties(
            &region,
            flash_props,
            restore_unwritten_bytes,
        )?;

        self.regions.push(HostLoadedRegion {
            region,
            data: FlashData::Raw(layout),
        });
        Ok(())
    }

    /// Returns the name of the flash algorithm.
    pub fn algorithm_name(&self) -> &str {
        &self.flash_algorithm.name
    }

    /// Host-side flashers don't support double buffering.
    pub(super) fn double_buffering_supported(&self) -> bool {
        false
    }

    /// Check if chip erase is supported.
    pub(super) fn is_chip_erase_supported(&self, _session: &Session) -> bool {
        // Host-side flash sequences always support erase_all
        true
    }

    /// Run chip erase via the debug flash sequence.
    pub(super) fn run_erase_all(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
    ) -> Result<(), FlashError> {
        tracing::info!("Host-side: Running chip erase");

        // Get the ARM debug interface
        let interface = session
            .get_arm_interface()
            .map_err(|e| FlashError::Core(e.into()))?;

        self.flash_sequence
            .erase_all(interface)
            .map_err(|e| FlashError::ChipEraseFailed {
                source: Box::new(e),
            })?;

        progress.finished_erasing();
        Ok(())
    }

    /// Program flash via the debug flash sequence.
    pub(super) fn program(
        &mut self,
        session: &mut Session,
        progress: &mut FlashProgress<'_>,
        _restore_unwritten_bytes: bool,
        _enable_double_buffering: bool,
        skip_erasing: bool,
        verify: bool,
    ) -> Result<(), FlashError> {
        tracing::debug!("Host-side: Starting program procedure");

        // Get the ARM debug interface
        let interface = session
            .get_arm_interface()
            .map_err(|e| FlashError::Core(e.into()))?;

        // Process each region
        for region in &self.regions {
            let layout = region.flash_layout();

            // Erase sectors if not skipping
            if !skip_erasing {
                tracing::debug!("Host-side: Erasing sectors");
                for sector in layout.sectors() {
                    tracing::debug!(
                        "Host-side: Erasing sector at 0x{:08X} ({} bytes)",
                        sector.address(),
                        sector.size()
                    );
                    self.flash_sequence
                        .erase_sector(interface, sector.address())
                        .map_err(|e| FlashError::EraseFailed {
                            sector_address: sector.address(),
                            source: Box::new(e),
                        })?;
                }
                progress.finished_erasing();
            }

            // Program pages
            tracing::debug!("Host-side: Programming pages");
            let mut t = Instant::now();
            for page in layout.pages() {
                tracing::debug!(
                    "Host-side: Programming page at 0x{:08X} ({} bytes)",
                    page.address(),
                    page.data().len()
                );
                self.flash_sequence
                    .program(interface, page.address(), page.data())
                    .map_err(|e| FlashError::PageWrite {
                        page_address: page.address(),
                        source: Box::new(e),
                    })?;

                progress.page_programmed(page.size() as u64, t.elapsed());
                t = Instant::now();
            }
            progress.finished_programming();

            // Verify if requested
            if verify {
                tracing::debug!("Host-side: Verifying");
                for page in layout.pages() {
                    let verified = self
                        .flash_sequence
                        .verify(interface, page.address(), page.data())
                        .map_err(|e| FlashError::Core(e.into()))?;

                    if !verified {
                        tracing::error!(
                            "Host-side: Verification failed at address 0x{:08X}",
                            page.address()
                        );
                        return Err(FlashError::Verify);
                    }
                }
            }
        }

        Ok(())
    }

    /// Verify flash contents against expected data.
    ///
    /// Returns `true` if all pages verify successfully, `false` otherwise.
    pub(super) fn verify(
        &self,
        session: &mut Session,
        _progress: &mut FlashProgress<'_>,
        _ignore_filled: bool,
    ) -> Result<bool, FlashError> {
        tracing::debug!("Host-side: Starting verify procedure");

        // Get the ARM debug interface
        let interface = session
            .get_arm_interface()
            .map_err(|e| FlashError::Core(e.into()))?;

        for region in &self.regions {
            let layout = region.flash_layout();
            for page in layout.pages() {
                let verified = self
                    .flash_sequence
                    .verify(interface, page.address(), page.data())
                    .map_err(|e| FlashError::Core(e.into()))?;

                if !verified {
                    tracing::error!(
                        "Host-side: Verification failed at address 0x{:08X}",
                        page.address()
                    );
                    return Ok(false);
                }
            }
        }

        Ok(true)
    }
}
