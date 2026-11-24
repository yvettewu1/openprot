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
use super::types::InitMode;

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

    #[must_use]
    pub const fn dma(bus: u8) -> Self {
        Self {
            bus,
            io_mode: IoMode::Dma,
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
    init_mode: Option<InitMode>,
    board_config: Option<super::config::BusConfig>,
    programmed_timing: Option<super::aspeed::PhyTiming>,
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
            init_mode: None,
            board_config: None,
            programmed_timing: None,
        })
    }

    /// Construct from the firmware bus table. BSP pins/clocks/cache setup must
    /// precede init_configured(). Disabled bus nodes are rejected without MMIO.
    ///
    /// # Safety
    /// Caller must own and serialize access to this physical bus.
    pub unsafe fn from_board(bus: u8, yield_us: Y) -> Result<Self> {
        let board = super::config::bus_config(bus)?;
        unsafe { Self::from_board_config(bus, board, yield_us) }
    }

    /// Use the board's role/timing/resources with an explicit transport selection.
    /// Selection is construction-time only; this never switches a running bus.
    ///
    /// # Safety
    /// Same unique ownership and BSP setup requirements as from_board().
    pub unsafe fn from_board_with_io(bus: u8, io_mode: IoMode, yield_us: Y) -> Result<Self> {
        let board = super::config::bus_config(bus)?.with_io_mode(io_mode)?;
        unsafe { Self::from_board_config(bus, board, yield_us) }
    }

    unsafe fn from_board_config(
        bus: u8,
        board: super::config::BusConfig,
        yield_us: Y,
    ) -> Result<Self> {
        let mut core = unsafe {
            Self::new(
                I3cHciConfig {
                    bus,
                    io_mode: board.io_mode,
                    data_big_endian: board.data_big_endian,
                },
                yield_us,
            )
        }?;
        core.board_config = Some(board);
        Ok(core)
    }

    pub fn board_config(&self) -> Option<super::config::BusConfig> {
        self.board_config
    }

    pub(crate) fn programmed_timing(&self) -> Option<super::aspeed::PhyTiming> {
        self.programmed_timing
    }

    /// Initialize the configured role and calculate/program its PHY timing.
    /// Does not enable NVIC or construct a controller/target session.
    pub fn init_configured(&mut self) -> Result<()> {
        use super::config::BusRole;
        let config = self.board_config.ok_or(I3cHciError::InvalidArgs)?;
        config.validate()?;
        match config.role {
            BusRole::Controller { dynamic_addr } => self.init_primary(dynamic_addr)?,
            BusRole::Secondary { .. } => self.init_secondary()?,
            BusRole::Target { .. } => self.init_target()?,
        }
        self.disable_irq_signals();
        Ok(())
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
        self.init_with_mode(None, None)
    }

    pub fn init_primary(&mut self, address: u8) -> Result<()> {
        if !super::daa::valid_i3c_address(address) {
            return Err(I3cHciError::InvalidArgs);
        }

        self.init_with_mode(Some(InitMode::Primary), Some(address))
    }

    /// Logical role selected by successful initialization, independent of bus.
    pub fn init_mode(&self) -> Option<InitMode> {
        self.init_mode
    }

    pub fn init_target(&mut self) -> Result<()> {
        self.init_with_mode(Some(InitMode::Target), None)
    }

    pub fn init_secondary(&mut self) -> Result<()> {
        self.init_with_mode(Some(InitMode::Secondary), None)
    }

    fn init_with_mode(&mut self, mode: Option<InitMode>, primary_addr: Option<u8>) -> Result<()> {
        // A failed reinitialization must not retain a previous role.
        self.init_mode = None;
        self.programmed_timing = None;
        // Validate every timing field before resetting hardware. Clock/divider
        // configuration belongs to BSP and must remain stable while the bus runs.
        let timing = if let Some(board) = self.board_config {
            board.validate()?;
            let input_hz = match board.timing.input_clock_hz {
                Some(hz) => hz,
                None => {
                    let (hpll, selection) = self.regs.i3c_clock_registers();
                    super::aspeed::i3c_input_clock_hz(hpll, selection)?
                }
            };
            Some(super::aspeed::PhyTiming::calculate(
                board.timing,
                board.scl_hz,
                input_hz,
            )?)
        } else {
            None // Explicit low-level initialization retains BSP-owned timing.
        };
        self.detect_version()?;
        self.discover_sections()?;
        self.soft_reset()?;
        self.setup_interrupts();
        self.setup_endianness()?;
        self.select_cmd_ops()?;
        self.select_io_ops()?;
        if let Some(mode) = mode {
            self.regs.aspeed_init(mode);

            if let Some(address) = primary_addr {
                // ASPEED Zephyr vendor init and program_master_addr precede BUS_ENABLE.
                self.regs.write_controller_device_addr(
                    constants::MASTER_DYNAMIC_ADDR_VALID
                        | (u32::from(address) << constants::MASTER_DYNAMIC_ADDR_SHIFT),
                );
                // Normal MIPI I3C transfers include the broadcast-address
                // preamble. Zephyr and Linux enable IBA for a primary controller;
                // leaving the reset default is appropriate only for other contexts
                // such as JESD403, which this initialization path does not select.
                self.regs.set_hc_control(
                    constants::HC_CONTROL_HOT_JOIN_CTRL | constants::HC_CONTROL_IBA_INCLUDE,
                );
            }
        }
        if let Some(timing) = timing {
            self.regs.aspeed_program_timing(&timing)?;
        }
        self.configure_polling_status();
        self.enable_bus()?;
        self.programmed_timing = timing;
        self.init_mode = mode;
        Ok(())
    }

    /// Make PIO status visible to software without routing interrupts to the CPU.
    /// The IRQ build may enable signaling later with `enable_irq_signals`.
    fn configure_polling_status(&self) {
        self.regs.write_pio_intr_signal_enable(0);
        if self.selected_io == Some(IoMode::Pio) {
            // ASPEED Zephyr hci_pio_init: half-FIFO data thresholds,
            // one command/response, no start gate. G7 needs both IBI
            // thresholds at one so short MDB notifications become visible.
            let size = self.regs.pio_queue_size();
            let mut rx = (size >> 16) & 0xff;
            let mut tx = (size >> 24) & 0xff;
            if self.version.is_some_and(|v| v.major == 1) {
                rx = rx.saturating_sub(1);
                tx = tx.saturating_sub(1);
            }
            self.regs
                .write_pio_data_buffer_thld_ctrl(((rx & 7) << 8) | (tx & 7));
            self.regs
                .write_pio_queue_thld_ctrl((1 << 24) | (1 << 16) | 0x0101);
            self.regs.write_pio_intr_status(u32::MAX);
            self.regs.write_pio_intr_status_enable(u32::MAX);
        }
    }

    fn enable_bus(&self) -> Result<()> {
        // MIPI I3C HCI HC_CONTROL.BUS_ENABLE is bit 31.
        const BUS_ENABLE: u32 = 1 << 31;
        self.regs.set_hc_control(BUS_ENABLE);
        if (self.regs.hc_control() & BUS_ENABLE) == 0 {
            return Err(I3cHciError::Busy);
        }
        Ok(())
    }

    pub fn clear_hc_irq(&self) -> u32 {
        let status = self.regs.intr_status();

        if status == 0 {
            return status;
        }
        /*
         * INTR_STATUS is W1C. Latch before clearing so normal driver context can
         * decide whether recovery is required.
         */

        self.regs.write_intr_status(status);
        status
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

        if self.selected_io == Some(IoMode::Pio) {
            self.regs.write_pio_intr_status_enable(u32::MAX);
            self.regs.write_pio_intr_signal_enable(pio_irqs);
        } else {
            // DMA ring interrupt signaling is configured by Ring::init.
            // Do not also signal the shared PIO FIFO interrupts in DMA mode.
            self.regs.write_pio_intr_signal_enable(0);
        }
    }

    pub fn disable_irq_signals(&self) {
        self.regs.write_intr_signal_enable(0);
        self.regs.write_pio_intr_signal_enable(0);
        if self.selected_io == Some(IoMode::Dma) {
            self.regs.write_dma_ring_intr_signal_enable(0);
        }
    }

    pub fn resume(&self) {
        self.regs.set_hc_control(constants::HC_CONTROL_RESUME);
    }

    pub fn abort(&mut self) -> Result<()> {
        self.regs.set_hc_control(constants::HC_CONTROL_ABORT);
        self.wait_hc_control_clear(constants::HC_CONTROL_ABORT)
    }

    pub fn reset_pio_queues(&mut self) -> Result<()> {
        self.reset_queues(
            constants::RX_FIFO_RST | constants::TX_FIFO_RST | constants::RESP_QUEUE_RST,
        )
    }

    pub fn reset_all_queues(&mut self) -> Result<()> {
        self.reset_queues(
            constants::CMD_QUEUE_RST
                | constants::RESP_QUEUE_RST
                | constants::RX_FIFO_RST
                | constants::TX_FIFO_RST
                | constants::IBI_QUEUE_RST,
        )
    }

    pub fn recover_pio(&mut self) -> Result<()> {
        self.abort()?;
        self.reset_pio_queues()?;
        self.resume();
        Ok(())
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
        self.wait_reset_bits_clear(constants::SOFT_RST)
    }

    fn reset_queues(&mut self, mask: u32) -> Result<()> {
        self.regs.write_reset_control(mask);
        self.wait_reset_bits_clear(mask)
    }

    fn wait_reset_bits_clear(&mut self, mask: u32) -> Result<()> {
        self.wait_register_clear(I3cHciRegisters::reset_control, mask)
    }

    fn wait_hc_control_clear(&mut self, mask: u32) -> Result<()> {
        self.wait_register_clear(I3cHciRegisters::hc_control, mask)
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
        let descriptor = select_command_descriptor(self.caps)?;
        self.command_descriptor = Some(descriptor);

        match descriptor {
            CommandDescriptor::V1 | CommandDescriptor::V2 => Ok(()),
        }
    }

    fn select_io_ops(&mut self) -> Result<()> {
        match self.config.io_mode {
            IoMode::Pio => self.select_pio(),
            IoMode::Dma => self.select_dma(),
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

    fn select_dma(&mut self) -> Result<()> {
        if self.sections.ring_headers_offset == 0 {
            return Err(I3cHciError::Unsupported);
        }

        super::dma::init_rhs(&self.regs, self.sections, 1)?;
        self.regs.clear_hc_control(constants::HC_CONTROL_PIO_MODE);
        self.selected_io = Some(IoMode::Dma);
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

impl<Y: FnMut(u32)> I3cHciCore<Y> {
    fn wait_register_clear(&mut self, read: fn(&I3cHciRegisters) -> u32, mask: u32) -> Result<()> {
        super::completion::poll(
            constants::MIPI_I3C_HCI_RESET_TIMEOUT_US,
            || {
                if read(&self.regs) & mask == 0 {
                    return Ok(Some(()));
                }
                (self.yield_us)(1);
                Ok(None)
            },
            I3cHciError::Timeout,
        )
    }
}
