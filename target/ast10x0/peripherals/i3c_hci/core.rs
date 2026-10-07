// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Core AST1080 MIPI I3C HCI init/probe logic.
//!
//! This follows Zephyr mipi_hci/core.c for the hardware bring-up sequence:
//! detect version, discover sections, soft reset, setup interrupts, setup
//! endianness, select command descriptor format, then select PIO transfer mode.

use super::constants;
use super::error::{I3cHciError, Result};
use super::irq::IsrCtx;
use super::registers::I3cHciRegisters;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IoMode {
    Pio,
    Dma,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandDescriptor {
    V1,
    V2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciVersion {
    pub major: u8,
    pub minor: u8,
    pub revision: u8,
    pub raw: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TableSection {
    pub offset: u16,
    pub entries: u16,
    pub entry_size: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HciSections {
    pub dat: TableSection,
    pub dct: TableSection,
    pub ring_headers_offset: u16,
    pub pio_offset: u16,
    pub ext_caps_offset: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I3cHciConfig {
    pub bus: u8,
    pub io_mode: IoMode,
    pub data_big_endian: bool,
}

impl I3cHciConfig {
    #[must_use]
    pub const fn pio(bus: u8) -> Self {
        Self {
            bus,
            io_mode: IoMode::Pio,
            data_big_endian: false,
        }
    }
}

pub struct I3cHciCore<Y: FnMut(u32)> {
    regs: I3cHciRegisters,
    config: I3cHciConfig,
    yield_us: Y,
    version: Option<HciVersion>,
    caps: u32,
    sections: HciSections,
    command_descriptor: Option<CommandDescriptor>,
    selected_io: Option<IoMode>,
}

impl<Y: FnMut(u32)> I3cHciCore<Y> {
    /// Create the core object for one bus without touching hardware.
    ///
    /// # Safety
    /// Caller must ensure unique, serialized access to the selected I3C HCI bus.
    pub unsafe fn new(config: I3cHciConfig, yield_us: Y) -> Result<Self> {
        let regs = unsafe { I3cHciRegisters::new(config.bus) }.ok_or(I3cHciError::InvalidBus)?;
        Ok(Self {
            regs,
            config,
            yield_us,
            version: None,
            caps: 0,
            sections: HciSections::default(),
            command_descriptor: None,
            selected_io: None,
        })
    }

    #[inline]
    pub fn regs(&self) -> &I3cHciRegisters {
        &self.regs
    }

    #[inline]
    pub fn version(&self) -> Option<HciVersion> {
        self.version
    }

    #[inline]
    pub fn capabilities(&self) -> u32 {
        self.caps
    }

    #[inline]
    pub fn sections(&self) -> HciSections {
        self.sections
    }

    #[inline]
    pub fn command_descriptor(&self) -> Option<CommandDescriptor> {
        self.command_descriptor
    }

    #[inline]
    pub fn selected_io(&self) -> Option<IoMode> {
        self.selected_io
    }

    pub fn init(&mut self) -> Result<()> {
        self.detect_version()?;
        self.discover_sections()?;
        self.soft_reset()?;
        self.setup_interrupts();
        self.setup_endianness()?;
        self.select_cmd_ops()?;
        self.select_io_ops()?;
        Ok(())
    }

    pub fn isr_ctx(&self) -> Result<IsrCtx> {
        let io_mode = self.selected_io.ok_or(I3cHciError::InvalidArgs)?;

        // SAFETY:
        // This creates a second facade over the same MMIO registers solely for
        // interrupt context. The ISR does not share Rust-owned mutable state with
        // this core object.
        unsafe { IsrCtx::new(self.config.bus, io_mode) }.ok_or(I3cHciError::InvalidBus)
    }

    pub fn enable_irq_signals(&self) {
        let status = self.regs.intr_status();

        if status != 0 {
            self.regs.write_intr_status(status);
        }

        self.regs
            .write_intr_status_enable(constants::MIPI_I3C_HCI_CORE_IRQS);
        self.regs
            .write_intr_signal_enable(constants::MIPI_I3C_HCI_CORE_IRQS);

        let pio_irqs = super::pio::STAT_RESP_READY | super::pio::STAT_ALL_ERRORS;

        self.regs.write_pio_intr_status_enable(pio_irqs);
        self.regs.write_pio_intr_signal_enable(pio_irqs);
    }

    pub fn disable_irq_signals(&self) {
        self.regs.write_intr_signal_enable(0);
        self.regs.write_pio_intr_signal_enable(0);
    }

    fn detect_version(&mut self) -> Result<()> {
        self.version = Some(parse_hci_version(self.regs.hci_version())?);
        Ok(())
    }

    fn discover_sections(&mut self) -> Result<()> {
        let version = self.version.ok_or(I3cHciError::InvalidArgs)?;
        self.caps = self.regs.hc_capabilities();
        self.sections = discover_sections_from_regs(
            version,
            self.regs.dat_section(),
            self.regs.dct_section(),
            self.regs.ring_headers_section(),
            self.regs.pio_section(),
            self.regs.ext_caps_section(),
        )?;
        Ok(())
    }

    fn soft_reset(&mut self) -> Result<()> {
        self.wait_reset_clear()?;
        self.regs.write_reset_control(constants::SOFT_RST);
        self.wait_reset_clear()
    }

    fn wait_reset_clear(&mut self) -> Result<()> {
        let mut remaining = constants::MIPI_I3C_HCI_RESET_TIMEOUT_US;
        while remaining > 0 {
            if (self.regs.reset_control() & constants::SOFT_RST) == 0 {
                return Ok(());
            }
            (self.yield_us)(1);
            remaining -= 1;
        }
        Err(I3cHciError::Timeout)
    }

    fn setup_interrupts(&self) {
        self.regs.write_intr_signal_enable(0);
        self.regs.write_intr_status_enable(0xffff_fc00);
    }

    fn setup_endianness(&self) -> Result<()> {
        let control = self.regs.hc_control();
        if self.config.data_big_endian {
            if (control & constants::HC_CONTROL_DATA_BIG_ENDIAN) == 0 {
                self.regs
                    .set_hc_control(constants::HC_CONTROL_DATA_BIG_ENDIAN);
                if (self.regs.hc_control() & constants::HC_CONTROL_DATA_BIG_ENDIAN) == 0 {
                    return Err(I3cHciError::Unsupported);
                }
            }
        } else if (control & constants::HC_CONTROL_DATA_BIG_ENDIAN) != 0 {
            self.regs
                .clear_hc_control(constants::HC_CONTROL_DATA_BIG_ENDIAN);
            if (self.regs.hc_control() & constants::HC_CONTROL_DATA_BIG_ENDIAN) != 0 {
                return Err(I3cHciError::Unsupported);
            }
        }
        Ok(())
    }

    fn select_cmd_ops(&mut self) -> Result<()> {
        self.command_descriptor = Some(select_command_descriptor(self.caps)?);
        Ok(())
    }

    fn select_io_ops(&mut self) -> Result<()> {
        match self.config.io_mode {
            IoMode::Pio => self.select_pio(),
            IoMode::Dma => Err(I3cHciError::Unsupported),
        }
    }

    fn select_pio(&mut self) -> Result<()> {
        if self.sections.pio_offset == 0 {
            return Err(I3cHciError::Unsupported);
        }
        self.regs.set_hc_control(constants::HC_CONTROL_PIO_MODE);
        if self
            .version
            .is_some_and(|v| v.major > 1 || (v.major == 1 && v.minor > 0))
            && (self.regs.hc_control() & constants::HC_CONTROL_PIO_MODE) == 0
        {
            return Err(I3cHciError::Busy);
        }
        self.selected_io = Some(IoMode::Pio);
        Ok(())
    }
}

pub fn parse_hci_version(raw: u32) -> Result<HciVersion> {
    let version = HciVersion {
        major: ((raw >> 8) & 0x0f) as u8,
        minor: ((raw >> 4) & 0x0f) as u8,
        revision: (raw & 0x0f) as u8,
        raw,
    };

    match raw & !0x0f {
        0x100 | 0x110 | 0x200 => Ok(version),
        _ => Err(I3cHciError::Unsupported),
    }
}

pub fn select_command_descriptor(caps: u32) -> Result<CommandDescriptor> {
    match field(
        caps,
        constants::HC_CAP_CMD_SIZE_MASK,
        constants::HC_CAP_CMD_SIZE_SHIFT,
    ) {
        0 => Ok(CommandDescriptor::V1),
        1 => Ok(CommandDescriptor::V2),
        _ => Err(I3cHciError::InvalidArgs),
    }
}

pub fn discover_sections_from_regs(
    version: HciVersion,
    dat_reg: u32,
    dct_reg: u32,
    ring_headers_reg: u32,
    pio_reg: u32,
    ext_caps_reg: u32,
) -> Result<HciSections> {
    let size_in_dwords = version.major < 1 || (version.major == 1 && version.minor < 1);

    let dat_entry_size = if field(
        dat_reg,
        constants::DAT_ENTRY_SIZE_MASK,
        constants::DAT_ENTRY_SIZE_SHIFT,
    ) == 0
    {
        8
    } else {
        0
    };
    if dat_entry_size == 0 {
        return Err(I3cHciError::InvalidArgs);
    }
    let mut dat_entries = field(
        dat_reg,
        constants::DAT_TABLE_SIZE_MASK,
        constants::DAT_TABLE_SIZE_SHIFT,
    ) as u16;
    if size_in_dwords {
        dat_entries = ((4 * u32::from(dat_entries)) / u32::from(dat_entry_size)) as u16;
    }

    let dct_entry_size = if field(
        dct_reg,
        constants::DCT_ENTRY_SIZE_MASK,
        constants::DCT_ENTRY_SIZE_SHIFT,
    ) == 0
    {
        16
    } else {
        0
    };
    if dct_entry_size == 0 {
        return Err(I3cHciError::InvalidArgs);
    }
    let mut dct_entries = field(
        dct_reg,
        constants::DCT_TABLE_SIZE_MASK,
        constants::DCT_TABLE_SIZE_SHIFT,
    ) as u16;
    if size_in_dwords {
        dct_entries = ((4 * u32::from(dct_entries)) / u32::from(dct_entry_size)) as u16;
    }

    Ok(HciSections {
        dat: TableSection {
            offset: (dat_reg & constants::DAT_TABLE_OFFSET_MASK) as u16,
            entries: dat_entries,
            entry_size: dat_entry_size,
        },
        dct: TableSection {
            offset: (dct_reg & constants::DCT_TABLE_OFFSET_MASK) as u16,
            entries: dct_entries,
            entry_size: dct_entry_size,
        },
        ring_headers_offset: (ring_headers_reg & constants::RING_HEADERS_OFFSET_MASK) as u16,
        pio_offset: (pio_reg & constants::PIO_REGS_OFFSET_MASK) as u16,
        ext_caps_offset: (ext_caps_reg & constants::EXT_CAPS_OFFSET_MASK) as u16,
    })
}

#[inline]
fn field(value: u32, mask: u32, shift: u32) -> u32 {
    (value & mask) >> shift
}
