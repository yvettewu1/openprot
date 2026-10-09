// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Confined unsafe MMIO facade for AST1080 MIPI I3C HCI.

use core::marker::PhantomData;

use ast1080_pac as device;

use super::constants;
use super::types::{InitMode, TargetRole};

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
    pub fn set_bus_enable(&self, enable: bool) {
        if enable {
            self.set_hc_control(constants::HC_CONTROL_BUS_ENABLE);
        } else {
            self.clear_hc_control(constants::HC_CONTROL_BUS_ENABLE);
        }
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
        let ptr = unsafe { (self.regs as *mut u8).add(0x34).cast::<u32>() };
        unsafe { core::ptr::write_volatile(ptr, value) };
    }
    #[inline]
    pub fn reset_dct_index(&self) {
        self.write_dct_section(self.dct_section() & !constants::DCT_TABLE_INDEX_MASK);
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

    #[inline]
    pub fn pio_queue_size(&self) -> u32 {
        self.regs().hcipio000().read().bits()
    }
    #[inline]
    pub fn write_pio_command_queue_port(&self, value: u32) {
        self.regs().hcipio004().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_response_queue_port(&self) -> u32 {
        self.regs().hcipio008().read().bits()
    }
    #[inline]
    pub fn write_pio_tx_data_port(&self, value: u32) {
        self.regs().hcipio00c().write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn pio_rx_data_port(&self) -> u32 {
        self.regs().hcipio010().read().bits()
    }
    #[inline]
    pub fn pio_ibi_port(&self) -> u32 {
        self.regs().hcipio014().read().bits()
    }
    #[inline]
    pub fn write_pio_queue_thld_ctrl(&self, value: u32) {
        self.regs().hcipio018().write(|w| unsafe { w.bits(value) });
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
    pub fn write_pio_intr_status_enable(&self, value: u32) {
        self.regs().hcipio024().write(|w| unsafe { w.bits(value) });
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
    ) -> Option<*mut u32> {
        if section_offset == 0 || entry_size == 0 || word >= 4 {
            return None;
        }
        let byte_offset = usize::from(section_offset)
            + usize::from(index) * usize::from(entry_size)
            + usize::from(word) * core::mem::size_of::<u32>();
        Some(unsafe { (self.regs as *mut u8).add(byte_offset).cast::<u32>() })
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
            .map(|ptr| unsafe { core::ptr::read_volatile(ptr) })
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
            .map(|ptr| unsafe { core::ptr::write_volatile(ptr, value) })
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
    #[inline]
    pub fn rhs_intr_status(&self) -> u32 {
        self.regs().hcirhs030().read().bits()
    }
    #[inline]
    pub fn write_rhs_intr_status(&self, value: u32) {
        self.regs().hcirhs030().write(|w| unsafe { w.bits(value) });
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
    pub fn aspeed_init(&self, mode: InitMode) {
        self.write_aspeed_ctrl(constants::ASPEED_I3C_CTRL_INIT | mode as u32);
    }
    #[inline]
    pub fn aspeed_status(&self) -> u32 {
        self.regs().i3ccontrol004().read().bits()
    }
    #[inline]
    pub fn aspeed_ibi_terminate(&self, max_len: u16) {
        self.regs().i3ccontrol008().write(|w| unsafe {
            w.bits(constants::ASPEED_I3C_IBI_TERMINATE_EN | u32::from(max_len))
        });
    }
    #[inline]
    pub fn aspeed_set_slave_char_ctrl(&self, bcr: u8, dcr: u8, static_addr: Option<u8>) {
        let mut value = (u32::from(dcr) << constants::ASPEED_I3C_SLV_CHAR_CTRL_DCR_SHIFT)
            | (u32::from(bcr) << constants::ASPEED_I3C_SLV_CHAR_CTRL_BCR_SHIFT);
        if let Some(addr) = static_addr {
            value |= constants::ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_EN
                | (u32::from(addr) & constants::ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_MASK);
        }
        self.regs()
            .i3ccontrol0a0()
            .write(|w| unsafe { w.bits(value) });
    }
    #[inline]
    pub fn aspeed_set_slave_pid(&self, pid: u64) {
        self.regs()
            .i3ccontrol0a4()
            .write(|w| unsafe { w.bits(pid as u32) });
        self.regs()
            .i3ccontrol0a8()
            .write(|w| unsafe { w.bits(((pid >> 32) & 0xffff) as u32) });
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
    pub fn aspeed_target_dynamic_addr(&self) -> Option<u8> {
        let status = self.aspeed_status();
        if (status & constants::ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_VALID) == 0 {
            return None;
        }
        Some(
            ((status & constants::ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_MASK)
                >> constants::ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_SHIFT) as u8,
        )
    }
    #[inline]
    pub fn aspeed_target_role(&self) -> TargetRole {
        let status = self.aspeed_status();
        let secondary = (status
            & (constants::ASPEED_I3C_STS_MODE_SECONDARY_SLV
                | constants::ASPEED_I3C_STS_MODE_SECONDARY_MST
                | constants::ASPEED_I3C_STS_MODE_SECONDARY_SLV_TO_MST
                | constants::ASPEED_I3C_STS_MODE_SECONDARY_MST_TO_SLV))
            != 0;
        let target = secondary || (status & constants::ASPEED_I3C_STS_MODE_PURE_SLV) != 0;
        TargetRole { secondary, target }
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
