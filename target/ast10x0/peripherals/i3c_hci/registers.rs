// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Confined unsafe MMIO facade for AST1080 MIPI I3C HCI.

use core::marker::PhantomData;

use ast1080_pac as device;

pub struct I3cHciRegisters {
    regs: *const device::i3c::RegisterBlock,
    bus: u8,
    _not_send_sync: PhantomData<*const ()>,
}

impl I3cHciRegisters {
    /// Create a register facade for AST1080 I3C bus 0..8.
    ///
    /// # Safety
    /// Caller must serialize access to the selected I3C controller.
    #[must_use]
    pub const unsafe fn new(bus: u8) -> Option<Self> {
        let regs = match bus {
            0 => device::I3c::ptr(),
            1 => device::I3c1::ptr(),
            2 => device::I3c2::ptr(),
            3 => device::I3c3::ptr(),
            4 => device::I3c4::ptr(),
            5 => device::I3c5::ptr(),
            6 => device::I3c6::ptr(),
            7 => device::I3c7::ptr(),
            _ => return None,
        };
        Some(Self {
            regs,
            bus,
            _not_send_sync: PhantomData,
        })
    }

    #[inline]
    #[must_use]
    pub const fn bus(&self) -> u8 {
        self.bus
    }

    #[inline]
    fn regs(&self) -> &'static device::i3c::RegisterBlock {
        // SAFETY: new guarantees a valid register-block pointer; the caller
        // owns serialization for this MMIO block.
        unsafe { &*self.regs }
    }

    #[inline]
    pub fn hci_version(&self) -> u32 {
        self.regs().hcicapability000().read().bits()
    }
    #[inline]
    pub fn hc_control(&self) -> u32 {
        self.regs().hcicapability004().read().bits()
    }
    #[inline]
    pub fn write_hc_control(&self, value: u32) {
        self.regs()
            .hcicapability004()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn set_hc_control(&self, mask: u32) {
        self.write_hc_control(self.hc_control() | mask);
    }
    #[inline]
    pub fn clear_hc_control(&self, mask: u32) {
        self.write_hc_control(self.hc_control() & !mask);
    }

    #[inline]
    pub fn controller_device_addr(&self) -> u32 {
        self.regs().hcicapability008().read().bits()
    }
    #[inline]
    pub fn write_controller_device_addr(&self, value: u32) {
        self.regs()
            .hcicapability008()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn hc_capabilities(&self) -> u32 {
        self.regs().hcicapability00c().read().bits()
    }
    #[inline]
    pub fn reset_control(&self) -> u32 {
        self.regs().hcicapability010().read().bits()
    }
    #[inline]
    pub fn write_reset_control(&self, value: u32) {
        self.regs()
            .hcicapability010()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn present_state(&self) -> u32 {
        self.regs().hcicapability014().read().bits()
    }
    #[inline]
    pub fn intr_status(&self) -> u32 {
        self.regs().hcicapability020().read().bits()
    }
    #[inline]
    pub fn write_intr_status(&self, value: u32) {
        self.regs()
            .hcicapability020()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn write_intr_status_enable(&self, value: u32) {
        self.regs()
            .hcicapability024()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn write_intr_signal_enable(&self, value: u32) {
        self.regs()
            .hcicapability028()
            .write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dat_section(&self) -> u32 {
        self.regs().hcicapability030().read().bits()
    }
    #[inline]
    pub fn dct_section(&self) -> u32 {
        self.regs().hcicapability034().read().bits()
    }
    #[inline]
    pub fn write_dct_section(&self, value: u32) {
        self.regs()
            .hcicapability034()
            .write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn ring_headers_section(&self) -> u32 {
        self.regs().hcicapability038().read().bits()
    }
    #[inline]
    pub fn pio_section(&self) -> u32 {
        self.regs().hcicapability03c().read().bits()
    }
    #[inline]
    pub fn ext_caps_section(&self) -> u32 {
        self.regs().hcicapability040().read().bits()
    }
    #[inline]
    pub fn write_ibi_notify_ctrl(&self, value: u32) {
        self.regs()
            .hcicapability058()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn write_dev_ctx_base_lo(&self, value: u32) {
        self.regs()
            .hcicapability060()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn write_dev_ctx_base_hi(&self, value: u32) {
        self.regs()
            .hcicapability064()
            .write(|w| unsafe { w.bits(value) });
    }

    // PAC HCIPIO offsets follow HCI: command +0x00, response +0x04,
    // shared TX/RX data +0x08, IBI +0x0c, queue threshold +0x10,
    // data threshold +0x14, queue size +0x18. FIFO reads have side effects.
    #[inline]
    pub fn pio_data_buffer_thld_ctrl(&self) -> u32 {
        self.regs().hcipio014().read().bits()
    }
    #[inline]
    pub fn write_pio_data_buffer_thld_ctrl(&self, value: u32) {
        self.regs().hcipio014().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_queue_size(&self) -> u32 {
        self.regs().hcipio018().read().bits()
    }
    #[inline]
    pub fn write_pio_command_queue_port(&self, value: u32) {
        self.regs().hcipio000().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_response_queue_port(&self) -> u32 {
        self.regs().hcipio004().read().bits()
    }
    #[inline]
    pub fn write_pio_tx_data_port(&self, value: u32) {
        self.regs().hcipio008().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_rx_data_port(&self) -> u32 {
        self.regs().hcipio008().read().bits()
    }
    #[inline]
    pub fn pio_ibi_port(&self) -> u32 {
        self.regs().hcipio00c().read().bits()
    }
    #[inline]
    pub fn pio_queue_thld_ctrl(&self) -> u32 {
        self.regs().hcipio010().read().bits()
    }
    #[inline]
    pub fn write_pio_queue_thld_ctrl(&self, value: u32) {
        self.regs().hcipio010().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_intr_status(&self) -> u32 {
        self.regs().hcipio020().read().bits()
    }
    #[inline]
    pub fn write_pio_intr_status(&self, value: u32) {
        self.regs().hcipio020().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_intr_status_enable(&self) -> u32 {
        self.regs().hcipio024().read().bits()
    }
    #[inline]
    pub fn write_pio_intr_status_enable(&self, value: u32) {
        self.regs().hcipio024().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_intr_signal_enable(&self) -> u32 {
        self.regs().hcipio028().read().bits()
    }
    #[inline]
    pub fn write_pio_intr_signal_enable(&self, value: u32) {
        self.regs().hcipio028().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    fn table_word_ptr(
        &self,
        section_offset: u16,
        entry_size: u8,
        index: u8,
        word: u8,
    ) -> Option<*const device::i3c::Hcidatsingle000> {
        if section_offset == 0
            || entry_size == 0
            || word >= 4
            || usize::from(word) * 4 >= usize::from(entry_size)
        {
            return None;
        }
        let byte_offset = usize::from(section_offset)
            + usize::from(index) * usize::from(entry_size)
            + usize::from(word) * core::mem::size_of::<u32>();
        if byte_offset % 4 != 0 || byte_offset + 4 > 0x1000 {
            return None;
        }
        // PAC has no dynamic DAT array. Its raw DWORD register type is valid
        // at each aligned word in the hardware-reported table window.
        Some(unsafe { (self.regs as *const u8).add(byte_offset).cast() })
    }

    #[inline]
    pub fn table_word(
        &self,
        section_offset: u16,
        entry_size: u8,
        index: u8,
        word: u8,
    ) -> Option<u32> {
        self.table_word_ptr(section_offset, entry_size, index, word)
            .map(|ptr| unsafe { (&*ptr).read().bits() })
    }

    #[inline]
    pub fn write_table_word(
        &self,
        section_offset: u16,
        entry_size: u8,
        index: u8,
        word: u8,
        value: u32,
    ) -> Option<()> {
        self.table_word_ptr(section_offset, entry_size, index, word)
            .map(|ptr| unsafe {
                (&*ptr).write(|w| w.bits(value));
            })
    }

    #[inline]
    pub fn dat_entry0(&self) -> u32 {
        self.regs().hcidatsingle000().read().bits()
    }
    #[inline]
    pub fn write_dat_entry0(&self, value: u32) {
        self.regs()
            .hcidatsingle000()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn dct_entry_word(&self, word: u8) -> Option<u32> {
        match word {
            0 => Some(self.regs().hcidct000().read().bits()),
            1 => Some(self.regs().hcidct004().read().bits()),
            2 => Some(self.regs().hcidct008().read().bits()),
            3 => Some(self.regs().hcidct00c().read().bits()),
            _ => None,
        }
    }
    #[inline]
    pub fn rhs_control(&self) -> u32 {
        self.regs().hcirhs000().read().bits()
    }
    #[inline]
    pub fn write_rhs_control(&self, value: u32) {
        self.regs().hcirhs000().write(|w| unsafe { w.bits(value) });
    }
    /// Hardware RH0_OFFSET, checked against the fixed PAC layout by DMA init.
    #[inline]
    pub fn rhs_ring0_offset(&self) -> u32 {
        self.regs().hcirhs004().read().bits()
    }

    pub fn dma_descriptor_sizes(&self) -> (u8, u8) {
        let setup = self.regs().hcirhs030().read();
        (
            setup.regxferstructsize().bits(),
            setup.regrespstructsize().bits(),
        )
    }

    pub fn dma_ibi_status_stride(&self) -> u8 {
        self.regs()
            .hcirhs034()
            .read()
            .regibistatusstructsize()
            .bits()
    }

    pub fn aspeed_wdma_debug_low(&self) -> u32 {
        self.regs().i3ccontrol084().read().regringwdmadbglo().bits()
    }

    pub fn aspeed_rdma_debug_low(&self) -> u32 {
        self.regs().i3ccontrol094().read().regringrdmadbglo().bits()
    }

    #[inline]
    pub fn dma_ring_cr_setup(&self) -> u32 {
        self.regs().hcirhs030().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_cr_setup(&self, value: u32) {
        self.regs().hcirhs030().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_ibi_setup(&self) -> u32 {
        self.regs().hcirhs034().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_ibi_setup(&self, value: u32) {
        self.regs().hcirhs034().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_chunk_control(&self) -> u32 {
        self.regs().hcirhs038().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_chunk_control(&self, value: u32) {
        self.regs().hcirhs038().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_intr_status(&self) -> u32 {
        self.regs().hcirhs040().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_intr_status(&self, value: u32) {
        self.regs().hcirhs040().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_intr_status_enable(&self) -> u32 {
        self.regs().hcirhs044().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_intr_status_enable(&self, value: u32) {
        self.regs().hcirhs044().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_intr_signal_enable(&self) -> u32 {
        self.regs().hcirhs048().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_intr_signal_enable(&self, value: u32) {
        self.regs().hcirhs048().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_status(&self) -> u32 {
        self.regs().hcirhs050().read().bits()
    }

    #[inline]
    pub fn dma_ring_control(&self) -> u32 {
        self.regs().hcirhs054().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_control(&self, value: u32) {
        self.regs().hcirhs054().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_operation1(&self) -> u32 {
        self.regs().hcirhs058().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_operation1(&self, value: u32) {
        self.regs().hcirhs058().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_operation2(&self) -> u32 {
        self.regs().hcirhs05c().read().bits()
    }

    #[inline]
    pub fn dma_ring_cmd_base_lo(&self) -> u32 {
        self.regs().hcirhs060().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_cmd_base_lo(&self, value: u32) {
        self.regs().hcirhs060().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_cmd_base_hi(&self) -> u32 {
        self.regs().hcirhs064().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_cmd_base_hi(&self, value: u32) {
        self.regs().hcirhs064().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_resp_base_lo(&self) -> u32 {
        self.regs().hcirhs068().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_resp_base_lo(&self, value: u32) {
        self.regs().hcirhs068().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_resp_base_hi(&self) -> u32 {
        self.regs().hcirhs06c().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_resp_base_hi(&self, value: u32) {
        self.regs().hcirhs06c().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_ibi_status_base_lo(&self) -> u32 {
        self.regs().hcirhs070().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_ibi_status_base_lo(&self, value: u32) {
        self.regs().hcirhs070().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_ibi_status_base_hi(&self) -> u32 {
        self.regs().hcirhs074().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_ibi_status_base_hi(&self, value: u32) {
        self.regs().hcirhs074().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_ibi_data_base_lo(&self) -> u32 {
        self.regs().hcirhs078().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_ibi_data_base_lo(&self, value: u32) {
        self.regs().hcirhs078().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn dma_ring_ibi_data_base_hi(&self) -> u32 {
        self.regs().hcirhs07c().read().bits()
    }

    #[inline]
    pub fn write_dma_ring_ibi_data_base_hi(&self, value: u32) {
        self.regs().hcirhs07c().write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn aspeed_ctrl(&self) -> u32 {
        self.regs().i3ccontrol000().read().bits()
    }
    #[inline]
    pub fn write_aspeed_ctrl(&self, value: u32) {
        self.regs()
            .i3ccontrol000()
            .write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn aspeed_status(&self) -> u32 {
        self.regs().i3ccontrol004().read().bits()
    }
    #[inline]
    pub fn aspeed_ibi_terminate_control(&self) -> u32 {
        self.regs().i3ccontrol008().read().bits()
    }

    #[inline]
    pub fn aspeed_slave_status1(&self) -> u32 {
        self.regs().i3ccontrol0b4().read().bits()
    }
    #[inline]
    pub fn aspeed_slave_cap_ctrl(&self) -> u32 {
        self.regs().i3ccontrol0b0().read().bits()
    }
    #[inline]
    pub fn write_aspeed_slave_cap_ctrl(&self, value: u32) {
        self.regs()
            .i3ccontrol0b0()
            .write(|w| unsafe { w.bits(value) });
    }

    #[inline]
    pub fn aspeed_intr_summary(&self) -> u32 {
        self.regs().i3ccontrol0f0().read().bits()
    }
    #[inline]
    pub fn aspeed_intr_renew(&self) {
        self.regs().i3ccontrol0f4().write(|w| unsafe { w.bits(1) });
    }
    #[inline]
    pub fn ext_cap_word(&self, word: u8) -> Option<u32> {
        match word {
            0 => Some(self.regs().hciextcap000().read().bits()),
            1 => Some(self.regs().hciextcap004().read().bits()),
            2 => Some(self.regs().hciextcap008().read().bits()),
            3 => Some(self.regs().hciextcap00c().read().bits()),
            4 => Some(self.regs().hciextcap010().read().bits()),
            5 => Some(self.regs().hciextcap014().read().bits()),
            6 => Some(self.regs().hciextcap018().read().bits()),
            7 => Some(self.regs().hciextcap01c().read().bits()),
            8 => Some(self.regs().hciextcap020().read().bits()),
            9 => Some(self.regs().hciextcap024().read().bits()),
            _ => None,
        }
    }
}

impl I3cHciRegisters {
    pub(crate) fn phy_038(&self) -> u32 {
        self.regs().i3cphyctrlreg038().read().bits()
    }
    pub(crate) fn phy_03c(&self) -> u32 {
        self.regs().i3cphyctrlreg03c().read().bits()
    }
    pub(crate) fn phy_040(&self) -> u32 {
        self.regs().i3cphyctrlreg040().read().bits()
    }
    pub(crate) fn phy_068(&self) -> u32 {
        self.regs().i3cphyctrlreg068().read().bits()
    }
    pub(crate) fn phy_06c(&self) -> u32 {
        self.regs().i3cphyctrlreg06c().read().bits()
    }
    pub(crate) fn phy_070(&self) -> u32 {
        self.regs().i3cphyctrlreg070().read().bits()
    }
    pub(crate) fn phy_090(&self) -> u32 {
        self.regs().i3cphyctrlreg090().read().bits()
    }
    pub(crate) fn phy_0cc(&self) -> u32 {
        self.regs().i3cphyctrlreg0cc().read().bits()
    }
    /// Raw PAC access only; calculation and programming policy live in aspeed.rs.
    pub(crate) fn write_phy_timing_word(
        &self,
        offset: u16,
        value: u32,
    ) -> super::error::Result<()> {
        match offset {
            0x004 => self
                .regs()
                .i3cphyctrlreg004()
                .write(|w| unsafe { w.bits(value) }),
            0x008 => self
                .regs()
                .i3cphyctrlreg008()
                .write(|w| unsafe { w.bits(value) }),
            0x00c => self
                .regs()
                .i3cphyctrlreg00c()
                .write(|w| unsafe { w.bits(value) }),
            0x010 => self
                .regs()
                .i3cphyctrlreg010()
                .write(|w| unsafe { w.bits(value) }),
            0x014 => self
                .regs()
                .i3cphyctrlreg014()
                .write(|w| unsafe { w.bits(value) }),
            0x018 => self
                .regs()
                .i3cphyctrlreg018()
                .write(|w| unsafe { w.bits(value) }),
            0x01c => self
                .regs()
                .i3cphyctrlreg01c()
                .write(|w| unsafe { w.bits(value) }),
            0x020 => self
                .regs()
                .i3cphyctrlreg020()
                .write(|w| unsafe { w.bits(value) }),
            0x024 => self
                .regs()
                .i3cphyctrlreg024()
                .write(|w| unsafe { w.bits(value) }),
            0x028 => self
                .regs()
                .i3cphyctrlreg028()
                .write(|w| unsafe { w.bits(value) }),
            0x02c => self
                .regs()
                .i3cphyctrlreg02c()
                .write(|w| unsafe { w.bits(value) }),
            0x030 => self
                .regs()
                .i3cphyctrlreg030()
                .write(|w| unsafe { w.bits(value) }),
            0x034 => self
                .regs()
                .i3cphyctrlreg034()
                .write(|w| unsafe { w.bits(value) }),
            0x038 => self
                .regs()
                .i3cphyctrlreg038()
                .write(|w| unsafe { w.bits(value) }),
            0x03c => self
                .regs()
                .i3cphyctrlreg03c()
                .write(|w| unsafe { w.bits(value) }),
            0x040 => self
                .regs()
                .i3cphyctrlreg040()
                .write(|w| unsafe { w.bits(value) }),
            0x044 => self
                .regs()
                .i3cphyctrlreg044()
                .write(|w| unsafe { w.bits(value) }),
            0x048 => self
                .regs()
                .i3cphyctrlreg048()
                .write(|w| unsafe { w.bits(value) }),
            0x04c => self
                .regs()
                .i3cphyctrlreg04c()
                .write(|w| unsafe { w.bits(value) }),
            0x050 => self
                .regs()
                .i3cphyctrlreg050()
                .write(|w| unsafe { w.bits(value) }),
            0x054 => self
                .regs()
                .i3cphyctrlreg054()
                .write(|w| unsafe { w.bits(value) }),
            0x058 => self
                .regs()
                .i3cphyctrlreg058()
                .write(|w| unsafe { w.bits(value) }),
            0x05c => self
                .regs()
                .i3cphyctrlreg05c()
                .write(|w| unsafe { w.bits(value) }),
            0x060 => self
                .regs()
                .i3cphyctrlreg060()
                .write(|w| unsafe { w.bits(value) }),
            0x064 => self
                .regs()
                .i3cphyctrlreg064()
                .write(|w| unsafe { w.bits(value) }),
            0x068 => self
                .regs()
                .i3cphyctrlreg068()
                .write(|w| unsafe { w.bits(value) }),
            0x06c => self
                .regs()
                .i3cphyctrlreg06c()
                .write(|w| unsafe { w.bits(value) }),
            0x070 => self
                .regs()
                .i3cphyctrlreg070()
                .write(|w| unsafe { w.bits(value) }),
            0x074 => self
                .regs()
                .i3cphyctrlreg074()
                .write(|w| unsafe { w.bits(value) }),
            0x078 => self
                .regs()
                .i3cphyctrlreg078()
                .write(|w| unsafe { w.bits(value) }),
            0x07c => self
                .regs()
                .i3cphyctrlreg07c()
                .write(|w| unsafe { w.bits(value) }),
            0x080 => self
                .regs()
                .i3cphyctrlreg080()
                .write(|w| unsafe { w.bits(value) }),
            0x098 => self
                .regs()
                .i3cphyctrlreg098()
                .write(|w| unsafe { w.bits(value) }),
            0x0d8 => self
                .regs()
                .i3cphyctrlreg0d8()
                .write(|w| unsafe { w.bits(value) }),
            _ => return Err(super::error::I3cHciError::InvalidArgs),
        };
        Ok(())
    }

    /// Snapshot read-only SCU clock inputs. BSP must serialize clock changes.
    pub(crate) fn i3c_clock_registers(&self) -> (u32, u32) {
        // SAFETY: fixed PAC SCU address; these are read-only accesses, not a claim
        // of exclusive ownership or permission to change the shared clock tree.
        let scu = unsafe { &*device::Scu::ptr() };
        (scu.scu300().read().bits(), scu.scu284().read().bits())
    }

    pub(crate) fn aspeed_queue_pointers(&self) -> u32 {
        self.regs().i3ccontrol0d8().read().bits()
    }
    pub(crate) fn write_aspeed_ibi_terminate_control(&self, value: u32) {
        self.regs()
            .i3ccontrol008()
            .write(|w| unsafe { w.bits(value) });
    }
    pub(crate) fn write_aspeed_slave_char_ctrl(&self, value: u32) {
        self.regs()
            .i3ccontrol0a0()
            .write(|w| unsafe { w.bits(value) });
    }
    pub(crate) fn write_aspeed_slave_pid_low(&self, value: u32) {
        self.regs()
            .i3ccontrol0a4()
            .write(|w| unsafe { w.bits(value) });
    }
    pub(crate) fn write_aspeed_slave_pid_high(&self, value: u32) {
        self.regs()
            .i3ccontrol0a8()
            .write(|w| unsafe { w.bits(value) });
    }
    pub(crate) fn write_aspeed_daa_bitmap(&self, bank: u8, value: u32) {
        match bank {
            0 => self
                .regs()
                .i3ccontrol010()
                .write(|w| unsafe { w.bits(value) }),
            1 => self
                .regs()
                .i3ccontrol014()
                .write(|w| unsafe { w.bits(value) }),
            2 => self
                .regs()
                .i3ccontrol018()
                .write(|w| unsafe { w.bits(value) }),
            3 => self
                .regs()
                .i3ccontrol01c()
                .write(|w| unsafe { w.bits(value) }),
            _ => unreachable!("DAA bitmap bank"),
        };
    }
}
