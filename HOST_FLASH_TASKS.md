# Host-Side Flash Support — Outstanding Tasks

Branch: `host-flash-support`

## Blockers (must land before PR merges)

### #17 — Make DebugFlashSequence architecture-agnostic and scalable to toolbox-based devices
**Why blocking:** Currently `DebugFlashSequence` lives in `architecture/arm/sequences.rs` and its
methods take `&mut dyn ArmDebugInterface`, making host-side flash ARM-only. Any non-ARM target
with `flash_loader_type: host_side` silently fails with a misleading error. The routing code in
`loader.rs` explicitly pattern-matches on `DebugSequence::Arm`.

Additionally, the current design assumes flash programming goes through the live debug connection
(CC27xx/SACI model). For devices like CC35xx where flash is handled by an external program
(SimpleLink WiFi Toolbox) that requires exclusive probe access, two further abstractions are
needed: a `prepare_flash()` probe-release hook and a `program_image()` whole-image method.

**Background — CC35xx toolbox model:**
The CC35xx WiFi SoC has no native SACI interface. Flash programming is delegated entirely to the
SimpleLink WiFi Toolbox (CLI or REST API on port 7777). The toolbox manages its own XDS110
connection, so probe-rs must release the probe before the toolbox runs and re-acquire it after.
The toolbox also operates on complete firmware images (`factory_programming`), not page-by-page.
See `~/git/openocd/tcl/target/ti_cc35x1e.cfg` and `~/git/osprey-simplelink-wifi-toolbox` for
reference.

**Work:**
1. Move `DebugFlashSequence` to a central location (e.g. `flashing/host_sequence.rs`) with no
   dependency on the ARM module.
2. Replace `&mut dyn ArmDebugInterface` in trait method signatures with `&mut Session`.
   Architecture-specific access (e.g. SEC-AP for CC27xx, probe serial number for CC35xx toolbox)
   is obtained inside the vendor implementation via `session.get_arm_interface()` etc.
3. Add `debug_flash_sequence() -> Option<Arc<dyn DebugFlashSequence>>` to `RiscvDebugSequence`
   and `XtensaDebugSequence` with default `None`.
4. Add a delegating `debug_flash_sequence()` method to the `DebugSequence` enum dispatching to
   all three variants.
5. Update `loader.rs::prepare_plan()` to call
   `session.target().debug_sequence.debug_flash_sequence()` — no more architecture match.
6. Add `prepare_flash(&mut Session) -> Result<()>` lifecycle hook (default: no-op). Called by
   `HostSideFlasher` before any flash operations begin. CC35xx overrides this to release the
   probe connection before the toolbox runs. `finish_flash()` is the symmetric counterpart.
7. Add `program_image(&mut Session, regions: &[(&NvmRegion, &FlashLayout)]) -> Option<Result<()>>`
   optional method (default: `None`, falls back to per-page `program()` loop). CC35xx overrides
   this to write all pages to a temp file and invoke the toolbox in one shot. CC27xx is
   unaffected.
8. Add toolbox path / external command configuration — likely as an optional field in the target
   YAML or a probe-rs config file — so CC35xx implementations can locate the toolbox binary.
9. Update `CC23xxCC27xxFlashSequence` to implement the updated trait: pass `&mut Session`, call
   `session.get_arm_interface()` internally, no-op `prepare_flash` and `program_image`.

---

## High Priority (important for correctness / quality)

### #12 — Replace unwrap() calls in build_sectors_and_pages_from_properties
**Why:** Two `.unwrap()` calls on `Option<T>` in `builder.rs`. Logically infallible given the
invariants, but will panic on malformed input rather than returning a proper error. Add a new
`FlashError` variant (e.g. `InternalError`) or convert helpers to return `Result`.

### #7 — Implement FLASH_PROG_MAIN_PIPELINED for MAIN flash
**Why:** Current per-sector approach (`FLASH_PROG_MAIN_SECTOR` 0x0E) takes ~850ms per 2KB sector.
For a 1MB binary (512 sectors) this is ~7 minutes. Root cause: 1024 USB round-trips per sector
due to mandatory per-word TXD_FULL polling. The pipelined command (0x0F) uses an ISR on the
device that clears TXD_FULL almost instantly, allowing writes to be batched — eliminating the
USB latency multiplier and giving a ~20x speedup for MAIN flash.

**Work:** Collect all MAIN pages for a region, send one pipelined header, stream sector data
without per-word TXD_FULL polling (USB packet spacing provides natural pacing), read per-sector
responses. Requires restructuring `HostSideFlasher` or adding a `program_region()` hook to
`DebugFlashSequence`.

### #13 — Fix poll_rx_ctrl USB hammering
**Why:** `poll_rx_ctrl` uses a 5µs sleep between polls. While waiting for device programming
(~20ms) this fires ~4000 USB reads. Increase sleep to ~1ms to reduce unnecessary USB traffic.
Low impact on total time but reduces bus contention.

---

## Medium Priority (correctness improvements)

### #10 — Honor ignore_filled in HostSideFlasher::verify
**Why:** The `ignore_filled` parameter is silently ignored in `HostSideFlasher::verify()`. The
RAM-based flasher uses this to skip pages containing only the erased byte value (0xFF), avoiding
unnecessary verification round-trips. Host-side should do the same.

### #16 — Use NvmRegion in HostSideFlasher error messages
**Why:** `HostLoadedRegion` stores the `NvmRegion` but it's currently `#[allow(dead_code)]`.
Wire it into error paths to include the region name and address range in failures, e.g.
`"Failed to program CCFG region (0x4E020000..0x4E020800)"`.

### #9 — Skip EXTRA_NS register in smoke tester write test
**Why:** `test_register_write` skips `EXTRA` but not `EXTRA_NS`. On ARMv8-M TrustZone devices
(CC2745R10Q1) the non-secure extra register (RegisterId 35) does not support write-back,
causing a spurious test failure on every smoke test run. Add `"EXTRA_NS"` to the skip list.

---

## Low Priority (cleanup)

### #8 — Encapsulate pub(super) fields on HostSideFlasher
**Why:** `core_index`, `flash_algorithm`, and `regions` are `pub(super)` on `HostSideFlasher`
and accessed by field name in `loader.rs`. `algorithm_name()` already shows the right pattern.
Add accessor methods and remove direct field access to keep the module boundary clean.

### #11 — Remove unused FlashRegion enum from cc23xx_cc27xx.rs
**Why:** The `FlashRegion` enum (`Main`, `Ccfg`, `Scfg`) is defined but never used — region
dispatch is done inline via address comparisons against `CCFG_START`/`SCFG_START` constants.
Either remove it or use it consistently in the dispatch logic.

---

## Completed

- **#14** — Make RawFlashAlgorithm RAM fields optional (instructions, pc_program_page,
  pc_erase_sector, data_section_offset) with validation at assembly time.
- **#15** — Clean up DebugFlashSequence trait: `erase_sector` now optional with default
  `NotImplemented`; `flash_properties()` removed from trait.
