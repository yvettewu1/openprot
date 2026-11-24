// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! ASPEED policy and ordered setup sequences.
//! All PAC access stays in registers.rs.
use super::{
    constants,
    registers::I3cHciRegisters,
    types::{InitMode, TargetRole},
};
impl I3cHciRegisters {
    /// Apply a fully validated image while BUS_ENABLE is clear. DMA/IBI queues
    /// are not touched. Registers and bank offsets follow ASPEED Zephyr.
    pub(crate) fn aspeed_program_timing(&self, timing: &PhyTiming) -> super::error::Result<()> {
        if self.hc_control() & constants::HC_CONTROL_BUS_ENABLE != 0 {
            return Err(super::error::I3cHciError::Busy);
        }
        for (base, values) in [(0x008, timing.fm), (0x018, timing.fmp), (0x028, timing.od)] {
            for (i, value) in values.into_iter().enumerate() {
                self.write_phy_timing_word(base + i as u16 * 4, value)?;
            }
        }
        // Populate the command-selected bank and the default SDR0 bank used
        // outside that command, including IBI. Zephyr also initializes DDR here.
        for base in [0x038 + u16::from(timing.bank) * 12, 0x038, 0x074] {
            for (i, value) in timing.sdr.into_iter().enumerate() {
                self.write_phy_timing_word(base + i as u16 * 4, value)?;
            }
        }
        self.write_phy_timing_word(0x080, timing.prepare)?;
        self.write_phy_timing_word(0x0d8, timing.dap)?;
        self.write_phy_timing_word(0x098, 0)?; // Zephyr PHY initialization.
        if let Some(pullup) = timing.pullup {
            self.write_phy_timing_word(0x004, pullup)?;
        }
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        Ok(())
    }

    #[inline]
    pub fn aspeed_init(&self, mode: InitMode) {
        self.write_aspeed_ctrl(constants::ASPEED_I3C_CTRL_INIT | mode as u32);
    }

    #[inline]
    pub fn aspeed_prepare_daa(&self, dynamic_addr: u8) {
        // ASPEED Zephyr vendor_aspeed.c: prep_daa_step selects one address
        // in the corresponding 32-address DAA bitmap before each ENTDAA.
        let value = 1u32 << (dynamic_addr & 31);
        self.write_aspeed_daa_bitmap(dynamic_addr / 32, value);
    }

    #[inline]
    pub fn aspeed_ibi_terminate(&self, max_len: u16) {
        // This limit is shared by all targets. Match ASPEED Linux/Zephyr:
        // enabling another subscriber must not shorten an existing limit.
        let previous =
            self.aspeed_ibi_terminate_control() & constants::ASPEED_I3C_IBI_TERMINATE_LEN_MASK;
        let max_len = core::cmp::max(previous, u32::from(max_len));
        self.write_aspeed_ibi_terminate_control(constants::ASPEED_I3C_IBI_TERMINATE_EN | max_len);
    }

    #[inline]
    pub fn aspeed_set_slave_char_ctrl(&self, bcr: u8, dcr: u8, static_addr: Option<u8>) {
        let mut value = (u32::from(dcr) << constants::ASPEED_I3C_SLV_CHAR_CTRL_DCR_SHIFT)
            | (u32::from(bcr) << constants::ASPEED_I3C_SLV_CHAR_CTRL_BCR_SHIFT);
        if let Some(addr) = static_addr {
            value |= constants::ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_EN
                | (u32::from(addr) & constants::ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_MASK);
        }
        self.write_aspeed_slave_char_ctrl(value);
    }

    #[inline]
    pub fn aspeed_set_slave_pid(&self, pid: u64) {
        self.write_aspeed_slave_pid_low(pid as u32);
        self.write_aspeed_slave_pid_high(((pid >> 32) & 0xffff) as u32);
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
    pub fn set_bus_enable(&self, enable: bool) {
        if enable {
            self.set_hc_control(constants::HC_CONTROL_BUS_ENABLE);
        } else {
            self.clear_hc_control(constants::HC_CONTROL_BUS_ENABLE);
        }
    }

    #[inline]
    pub fn reset_dct_index(&self) {
        self.write_dct_section(self.dct_section() & !constants::DCT_TABLE_INDEX_MASK);
    }
}

/// Register image calculated before any PHY writes. All timing fields are 11 bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PhyTiming {
    pub(crate) input_hz: u32,
    pub(crate) requested_scl_hz: u32,
    pub(crate) bank: u8,
    fm: [u32; 4],
    fmp: [u32; 4],
    pub(crate) od: [u32; 4],
    pub(crate) sdr: [u32; 3],
    prepare: u32,
    dap: u32,
    pullup: Option<u32>,
}

/// Matches Zephyr clock_control_ast10x0_g2.c. Read-only: preserve BSP clocks.
pub(crate) fn i3c_input_clock_hz(hpll: u32, selection: u32) -> super::error::Result<u32> {
    use super::error::I3cHciError;
    if hpll & ((1 << 23) | (1 << 25)) != 0 {
        return Err(I3cHciError::InvalidArgs); // PLL off or held in reset.
    }
    let pll_hz = if hpll & (1 << 24) != 0 {
        25_000_000u64
    } else {
        25_000_000u64 * (u64::from(hpll & 0x1fff) + 1)
            / (u64::from((hpll >> 13) & 0x3f) + 1)
            / (u64::from((hpll >> 19) & 0xf) + 1)
    };
    let divider = [2u64, 2, 3, 4, 5, 6, 7, 8][((selection >> 23) & 7) as usize];
    let hz = pll_hz / divider;
    if hz == 0 || hz > 1_000_000_000 {
        return Err(I3cHciError::InvalidArgs);
    }
    Ok(hz as u32)
}

impl PhyTiming {
    /// ASPEED Zephyr populate_bus_timing + phy_init, with checked field widths.
    /// Nanosecond rounding intentionally matches Zephyr's integer clock period.
    pub(crate) fn calculate(
        config: super::config::TimingConfig,
        scl_hz: u32,
        input_hz: u32,
    ) -> super::error::Result<Self> {
        use super::error::I3cHciError;
        config.validate(scl_hz)?;
        if input_hz == 0 || input_hz > 1_000_000_000 || input_hz / scl_hz < 2 {
            return Err(I3cHciError::InvalidArgs);
        }
        let period_ns = 1_000_000_000u64.div_ceil(u64::from(input_hz));
        let closest = |ns: u32| ((u64::from(ns) + period_ns / 2) / period_ns) as u32;
        let minus_one = |ns: u32| closest(ns).saturating_sub(1);
        let round_up = |ns: u32| u64::from(ns).div_ceil(period_ns).saturating_sub(1) as u32;
        let pack = |high: u32, low: u32| {
            if high > 0x7ff || low > 0x7ff {
                Err(I3cHciError::InvalidArgs)
            } else {
                Ok((high << 16) | low)
            }
        };
        let (high, low) = if let Some(pp) = config.push_pull {
            (minus_one(pp.high_ns), minus_one(pp.low_ns))
        } else {
            let total = input_hz.div_ceil(scl_hz) - 2;
            let high = if config.pure_i3c_bus {
                ((u64::from(total) * 2) / 5) as u32
            } else {
                round_up(41)
            };
            (
                high,
                total.checked_sub(high).ok_or(I3cHciError::InvalidArgs)?,
            )
        };
        let scl = pack(high, low)?;
        let tbit = pack(high.max(round_up(60)), low.max(round_up(60)))?;
        let hold = closest(if config.sda_tx_hold_ns == 0 {
            10
        } else {
            config.sda_tx_hold_ns
        });
        let hold = pack(hold, hold)?;
        let od = config.open_drain.unwrap_or(super::config::SclPeriods {
            high_ns: 380,
            low_ns: 620,
        });
        let od_high = minus_one(od.high_ns);
        let od_low = minus_one(od.low_ns);
        let i2c = |high_ns, low_ns| -> super::error::Result<[u32; 4]> {
            let h = minus_one(high_ns);
            let l = minus_one(low_ns);
            let counts = pack(h, l)?;
            Ok([
                counts,
                counts,
                pack(h, h)?,
                pack(minus_one(10), minus_one(10))?,
            ])
        };
        Ok(Self {
            input_hz,
            requested_scl_hz: scl_hz,
            bank: super::cmd::i3c_sdr_mode(scl_hz),
            fm: i2c(1130, 1370)?,
            fmp: i2c(380, 620)?,
            od: [
                pack(high, high)?,
                pack(od_high, od_low)?,
                pack(od_high, od_high)?,
                hold,
            ],
            sdr: [scl, tbit, hold],
            prepare: pack(closest(16), low)?,
            dap: pack(minus_one(12), 0)?,
            pullup: (config.internal_pullup != 0).then_some(
                (1 << 28)
                    | (1 << 24)
                    | (u32::from(config.internal_pullup) << 8)
                    | u32::from(config.internal_pullup),
            ),
        })
    }
}

#[cfg(test)]
mod timing_tests {
    use super::super::config::{SclPeriods, TimingConfig};
    use super::*;

    #[test]
    fn pure_bus_uses_requested_rate_at_two_input_clocks() {
        let cfg = TimingConfig::automatic();
        let timing = PhyTiming::calculate(cfg, 1_000_000, 200_000_000).unwrap();
        assert_eq!(timing.bank, 4);
        assert_eq!(timing.sdr, [0x004f_0077, 0x004f_0077, 0x0002_0002]);
        assert_eq!(
            timing.fm,
            [0x00e1_0111, 0x00e1_0111, 0x00e1_00e1, 0x0001_0001]
        );
        assert_eq!(timing.od[1], 0x004b_007b);
        let timing = PhyTiming::calculate(cfg, 1_000_000, 500_000_000).unwrap();
        assert_eq!(timing.sdr, [0x00c7_012b, 0x00c7_012b, 0x0005_0005]);
    }

    #[test]
    fn fast_bus_keeps_tbit_minimum_and_selects_each_bank() {
        let cfg = TimingConfig::automatic();
        let timing = PhyTiming::calculate(cfg, 12_500_000, 200_000_000).unwrap();
        assert_eq!(timing.sdr, [0x0005_0009, 0x000b_000b, 0x0002_0002]);
        for (hz, bank) in [
            (12_500_000, 0),
            (8_000_000, 1),
            (6_000_000, 2),
            (4_000_000, 3),
            (2_000_000, 4),
        ] {
            assert_eq!(
                PhyTiming::calculate(cfg, hz, 200_000_000).unwrap().bank,
                bank
            );
        }
    }

    #[test]
    fn mixed_bus_and_explicit_periods_follow_zephyr() {
        let mut cfg = TimingConfig::automatic();
        cfg.pure_i3c_bus = false;
        let timing = PhyTiming::calculate(cfg, 1_000_000, 200_000_000).unwrap();
        assert_eq!(timing.sdr[0], 0x0008_00be);
        cfg.push_pull = Some(SclPeriods {
            high_ns: 500,
            low_ns: 500,
        });
        cfg.open_drain = Some(SclPeriods {
            high_ns: 1000,
            low_ns: 1500,
        });
        cfg.sda_tx_hold_ns = 20;
        cfg.internal_pullup = 3;
        let timing = PhyTiming::calculate(cfg, 1_000_000, 500_000_000).unwrap();
        assert_eq!(timing.sdr, [0x00f9_00f9, 0x00f9_00f9, 0x000a_000a]);
        assert_eq!(timing.od[1], 0x01f3_02ed);
        assert_eq!(timing.pullup, Some(0x1100_0303));
    }

    #[test]
    fn clock_decode_matches_hpll_divider_and_bypass() {
        // 25 MHz * 80 = 2 GHz HPLL, divided by 4 or 8.
        assert_eq!(i3c_input_clock_hz(79, 3 << 23).unwrap(), 500_000_000);
        assert_eq!(i3c_input_clock_hz(79, 7 << 23).unwrap(), 250_000_000);
        assert_eq!(i3c_input_clock_hz(1 << 24, 0).unwrap(), 12_500_000);
        assert!(i3c_input_clock_hz(1 << 23, 0).is_err());
        assert!(i3c_input_clock_hz(1 << 25, 0).is_err());
        // Include N and P divisors: 25 MHz * 80 / 2 / 2 / 4.
        assert_eq!(
            i3c_input_clock_hz(79 | (1 << 13) | (1 << 19), 3 << 23).unwrap(),
            125_000_000
        );
    }

    #[test]
    fn invalid_timing_is_rejected_before_register_programming() {
        let mut cfg = TimingConfig::automatic();
        assert!(PhyTiming::calculate(cfg, 0, 200_000_000).is_err());
        assert!(PhyTiming::calculate(cfg, 1_000_000, 0).is_err());
        assert!(PhyTiming::calculate(cfg, 13_000_000, 200_000_000).is_err());
        assert!(PhyTiming::calculate(cfg, 1, 200_000_000).is_err()); // 11-bit overflow.
        cfg.push_pull = Some(SclPeriods {
            high_ns: 0,
            low_ns: 500,
        });
        assert!(PhyTiming::calculate(cfg, 1_000_000, 200_000_000).is_err());
        cfg.push_pull = Some(SclPeriods {
            high_ns: u32::MAX,
            low_ns: 500,
        });
        assert!(PhyTiming::calculate(cfg, 1_000_000, 200_000_000).is_err());
        cfg.push_pull = None;
        cfg.sda_tx_hold_ns = u32::MAX;
        assert!(PhyTiming::calculate(cfg, 1_000_000, 200_000_000).is_err());
    }
}
