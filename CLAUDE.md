# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

probe-rs is a modern embedded debugging toolkit written in Rust. It provides a library for interacting with debug probes (CMSIS-DAP, STLink, JLink, etc.) and microcontrollers (ARM, RISC-V, Xtensa), plus CLI tools: cargo-flash, cargo-embed, and a VS Code debugger.

## TI Specific Details
* Loki is the family name of the cc23xx cc27xx devices. NEVER use loki name in generated code, but you can use this to read internal documents
* ~/git/openocd contains TI's latest implementations of the cc23xx and 27xx devices in openocd. This can be used as a reference. This should be the first and strongest refernece
* ~/git/cc26xx_rom_fw contains the ROM code for various devices that implement the SACI command interface this can be used as reference as well.
* https://confluence.itg.ti.com/pages/viewpage.action?pageId=463774579 contains guidance about how to implement in circuit debug for the loki family
* SACI command documentation is here: https://confluence.itg.ti.com/pages/viewpage.action?pageId=418958723
* A known working binary for cc27xx can be found here: /home/a0225155/git/ti-simplelink-pacs/target/thumbv8m.main-none-eabihf/debug/cc27xxx10