// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Firmware-owned I3C board configuration (the equivalent of bus DTS nodes).
//! Array index is the PAC hardware bus number. Enable only instances present
//! and connected on the board. Pins, clocks, voltage and cache setup remain BSP work.
use super::constants::MAX_BUSES;
use super::core::IoMode;
use super::daa::valid_i3c_address;
use super::error::{I3cHciError, Result};
use super::target::HciTargetConfig;
use super::types::InitMode;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusRole {
    Controller { dynamic_addr: u8 },
    Secondary { identity: HciTargetConfig },
    Target { identity: HciTargetConfig },
}

impl BusRole {
    pub const fn init_mode(self) -> InitMode {
        match self {
            Self::Controller { .. } => InitMode::Primary,
            Self::Secondary { .. } => InitMode::Secondary,
            Self::Target { .. } => InitMode::Target,
        }
    }
    pub const fn identity(self) -> Option<HciTargetConfig> {
        match self {
            Self::Controller { .. } => None,
            Self::Secondary { identity } | Self::Target { identity } => Some(identity),
        }
    }
}

/// Explicit high/low SCL periods, equivalent to Zephyr's *_scl_*_period_ns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SclPeriods {
    pub high_ns: u32,
    pub low_ns: u32,
}

/// Board PHY inputs. Clock gates/dividers must be stable before initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimingConfig {
    /// None reads AST1040 SCU HPLL/divider; Some supplies the actual PHY input Hz.
    /// This does not change the SCU clock source or divider.
    pub input_clock_hz: Option<u32>,
    /// False when legacy I2C devices share the wires (Zephyr mixed-bus duty cycle).
    pub pure_i3c_bus: bool,
    pub push_pull: Option<SclPeriods>,
    pub open_drain: Option<SclPeriods>,
    /// Zero selects Zephyr's 10 ns default.
    pub sda_tx_hold_ns: u32,
    /// ASPEED pull-up encoding, 0..7. Zero leaves SW pull-up override unchanged.
    pub internal_pullup: u8,
}

impl TimingConfig {
    pub const fn automatic() -> Self {
        Self {
            input_clock_hz: None,
            pure_i3c_bus: true,
            push_pull: None,
            open_drain: None,
            sda_tx_hold_ns: 0,
            internal_pullup: 0,
        }
    }

    pub(crate) fn validate(self, scl_hz: u32) -> Result<()> {
        if scl_hz == 0
            || scl_hz > 12_500_000
            || self
                .input_clock_hz
                .is_some_and(|hz| hz == 0 || hz > 1_000_000_000)
            || self.internal_pullup > 7
            || [self.push_pull, self.open_drain]
                .into_iter()
                .flatten()
                .any(|p| p.high_ns == 0 || p.low_ns == 0)
        {
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusConfig {
    pub enabled: bool,
    pub role: BusRole,
    /// Default transport. Tests may select another transport before initialization.
    pub io_mode: IoMode,
    pub data_big_endian: bool,
    /// Requested SDR frequency; selects the command bank and calculates PHY timing.
    pub scl_hz: u32,
    pub timing: TimingConfig,
    /// Maximum unsuccessful probes per phase, not elapsed microseconds.
    /// Applies to both PIO and DMA; PIO retains its one-microsecond yield hook.
    pub poll_budget: u32,
    /// Reserved DMA capacity, independent of the selected transport. Zero disables DMA.
    pub dma_payload_bytes: usize,
}

impl BusConfig {
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            role: BusRole::Controller { dynamic_addr: 8 },
            io_mode: IoMode::Pio,
            data_big_endian: false,
            scl_hz: 1_000_000,
            timing: TimingConfig::automatic(),
            poll_budget: 1_000_000,
            dma_payload_bytes: 0,
        }
    }
    pub const fn uses_dma(self) -> bool {
        self.enabled && matches!(self.io_mode, IoMode::Dma)
    }
    /// Resources stay reserved when an application selects PIO for this port.
    pub const fn has_dma_resources(self) -> bool {
        self.enabled && self.dma_payload_bytes != 0
    }

    /// Select a transport before initialization; preserve role, timing and memory.
    pub fn with_io_mode(mut self, io_mode: IoMode) -> Result<Self> {
        self.io_mode = io_mode;
        self.validate()?;
        Ok(self)
    }
    pub fn validate(self) -> Result<()> {
        if !self.enabled {
            return Err(I3cHciError::Unsupported);
        }
        self.timing.validate(self.scl_hz)?;
        if self.poll_budget == 0 {
            return Err(I3cHciError::InvalidArgs);
        }
        match self.role {
            BusRole::Controller { dynamic_addr } => {
                if !valid_i3c_address(dynamic_addr) {
                    return Err(I3cHciError::InvalidArgs);
                }
            }
            BusRole::Secondary { identity } | BusRole::Target { identity } => {
                identity.validate()?
            }
        }
        if self.dma_payload_bytes > 128 * 1024
            || self.dma_payload_bytes % 4 != 0
            || (self.uses_dma() && !self.has_dma_resources())
        {
            return Err(I3cHciError::InvalidArgs);
        }
        Ok(())
    }
}

/// Dedicated board ports shared by MCTP and loopback tests.
/// Select physical wiring here once; each test selects its own transport.
pub const CONTROLLER_BUS: u8 = 0;
pub const TARGET_BUS: u8 = 1;

/// Board roles, identities, timing and reserved transport resources.
/// Firmware engineers edit this table for their board/application. Slots 2..7
/// are disabled by default; unused/unavailable hardware must stay disabled.
pub const BUSES: [BusConfig; MAX_BUSES] = {
    let mut buses = [BusConfig::disabled(); MAX_BUSES];
    buses[CONTROLLER_BUS as usize] = BusConfig {
        enabled: true,
        role: BusRole::Controller { dynamic_addr: 8 },
        io_mode: IoMode::Dma,
        dma_payload_bytes: 48 * 1024,
        ..BusConfig::disabled()
    };
    buses[TARGET_BUS as usize] = BusConfig {
        enabled: true,
        role: BusRole::Secondary {
            identity: HciTargetConfig {
                pid: 0x1234_5678_9abc,
                bcr: 0x06,
                dcr: 0xcc,
                static_addr: None,
            },
        },
        io_mode: IoMode::Dma,
        dma_payload_bytes: 48 * 1024,
        ..BusConfig::disabled()
    };
    buses
};

pub fn bus_config(bus: u8) -> Result<BusConfig> {
    let config = *BUSES.get(bus as usize).ok_or(I3cHciError::InvalidBus)?;
    config.validate()?;
    Ok(config)
}

pub(crate) const fn dma_index(bus: usize) -> Option<usize> {
    dma_index_in(&BUSES, bus)
}

pub(crate) const fn dma_payload_offset(bus: usize) -> usize {
    dma_payload_offset_in(&BUSES, bus)
}

const fn dma_index_in(buses: &[BusConfig; MAX_BUSES], bus: usize) -> Option<usize> {
    if bus >= MAX_BUSES || !buses[bus].has_dma_resources() {
        return None;
    }
    let mut count = 0;
    let mut i = 0;
    while i < bus {
        if buses[i].has_dma_resources() {
            count += 1;
        }
        i += 1;
    }
    Some(count)
}

const fn dma_payload_offset_in(buses: &[BusConfig; MAX_BUSES], bus: usize) -> usize {
    let mut offset = 0;
    let mut i = 0;
    while i < bus {
        if buses[i].has_dma_resources() {
            offset += (buses[i].dma_payload_bytes + 63) & !63;
        }
        i += 1;
    }
    offset
}

pub(crate) const DMA_INSTANCE_COUNT: usize = {
    let mut count = 0;
    let mut i = 0;
    while i < MAX_BUSES {
        if BUSES[i].has_dma_resources() {
            count += 1;
        }
        i += 1;
    }
    count
};
pub(crate) const DMA_TOTAL_PAYLOAD_BYTES: usize = dma_payload_offset(MAX_BUSES);

const _: () = {
    assert!((CONTROLLER_BUS as usize) < MAX_BUSES);
    assert!((TARGET_BUS as usize) < MAX_BUSES);
    assert!(CONTROLLER_BUS != TARGET_BUS);
    let mut i = 0;
    while i < MAX_BUSES {
        let cfg = BUSES[i];
        if cfg.has_dma_resources() {
            assert!(cfg.dma_payload_bytes > 0 && cfg.dma_payload_bytes <= 128 * 1024);
            assert!(cfg.dma_payload_bytes % 4 == 0);
        } else {
            assert!(cfg.dma_payload_bytes == 0);
        }
        i += 1;
    }
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nonadjacent_dma_buses_have_independent_compact_storage() {
        let mut buses = [BusConfig::disabled(); MAX_BUSES];
        buses[2] = BusConfig {
            enabled: true,
            io_mode: IoMode::Dma,
            dma_payload_bytes: 4096,
            ..BusConfig::disabled()
        };
        buses[6] = BusConfig {
            enabled: true,
            io_mode: IoMode::Dma,
            dma_payload_bytes: 8192,
            role: BusRole::Target {
                identity: HciTargetConfig {
                    pid: 1,
                    bcr: 0,
                    dcr: 0,
                    static_addr: None,
                },
            },
            ..BusConfig::disabled()
        };
        assert_eq!(dma_index_in(&buses, 0), None);
        assert_eq!(dma_index_in(&buses, 2), Some(0));
        assert_eq!(dma_index_in(&buses, 6), Some(1));
        assert_eq!(dma_payload_offset_in(&buses, 2), 0);
        assert_eq!(dma_payload_offset_in(&buses, 6), 4096);
        assert_eq!(dma_payload_offset_in(&buses, MAX_BUSES), 12288);
    }

    #[test]
    fn rejects_disabled_nodes_and_inconsistent_transport_budgets() {
        let mut config = BusConfig::disabled();
        assert!(config.validate().is_err());
        config.enabled = true;
        assert!(config.validate().is_ok());
        config.dma_payload_bytes = 4;
        assert!(config.validate().is_ok()); // PIO can retain reserved DMA capacity.
        config.io_mode = IoMode::Dma;
        assert!(config.validate().is_ok());
        config.dma_payload_bytes = 3;
        assert!(config.validate().is_err());
    }
    #[test]
    fn transport_selection_keeps_board_resources_and_identity() {
        let mut buses = BUSES;
        let controller = CONTROLLER_BUS as usize;
        let target = TARGET_BUS as usize;
        let offset = dma_payload_offset_in(&buses, target);
        let index = dma_index_in(&buses, target);
        let original = buses[target];
        buses[controller] = buses[controller].with_io_mode(IoMode::Pio).unwrap();
        buses[target] = buses[target].with_io_mode(IoMode::Pio).unwrap();
        assert_eq!(dma_payload_offset_in(&buses, target), offset);
        assert_eq!(dma_index_in(&buses, target), index);
        assert_eq!(buses[target].role, original.role);
        assert_eq!(buses[target].timing, original.timing);
        assert_eq!(buses[target].with_io_mode(IoMode::Dma).unwrap(), original);
    }

    #[test]
    fn selecting_dma_requires_reserved_memory() {
        let pio_only = BusConfig {
            enabled: true,
            ..BusConfig::disabled()
        };
        assert!(pio_only.with_io_mode(IoMode::Pio).is_ok());
        assert!(pio_only.with_io_mode(IoMode::Dma).is_err());
        assert!(BusConfig::disabled().with_io_mode(IoMode::Pio).is_err());
    }
}
