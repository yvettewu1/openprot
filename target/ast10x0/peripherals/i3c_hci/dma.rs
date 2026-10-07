// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Minimal MIPI I3C HCI DMA/RHS bring-up support.
//!
//! This is intentionally only the RHS foundation. Transfer descriptor rings are
//! added later once the ring memory ownership model is in place.

use super::core::HciSections;
use super::error::{I3cHciError, Result};
use super::registers::I3cHciRegisters;

pub const RHS_CONTROL_MAX_HEADER_COUNT_SHIFT: u32 = 0;
pub const RHS_CONTROL_MAX_HEADER_COUNT_MASK: u32 = 0x0f << RHS_CONTROL_MAX_HEADER_COUNT_SHIFT;
pub const RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT: u32 = 4;
pub const RHS_CONTROL_MAX_HEADER_COUNT_CAP_MASK: u32 =
    0x0f << RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT;
pub const RHS_CONTROL_HEADER_SIZE_SHIFT: u32 = 16;
pub const RHS_CONTROL_HEADER_SIZE_MASK: u32 = 0xff << RHS_CONTROL_HEADER_SIZE_SHIFT;
pub const RHS_CONTROL_PREAMBLE_SIZE_SHIFT: u32 = 24;
pub const RHS_CONTROL_PREAMBLE_SIZE_MASK: u32 = 0xff << RHS_CONTROL_PREAMBLE_SIZE_SHIFT;

pub const RHS_INTR_RING0: u32 = 1 << 0;
pub const RHS_INTR_ALL_RINGS: u32 = 0xff;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RhsInfo {
    pub preamble_size: u8,
    pub header_size: u8,
    pub max_header_count_cap: u8,
    pub driver_header_count: u8,
    pub ring_headers_offset: u16,
}

#[must_use]
pub const fn parse_rhs_control(control: u32, ring_headers_offset: u16) -> RhsInfo {
    RhsInfo {
        preamble_size: ((control & RHS_CONTROL_PREAMBLE_SIZE_MASK)
            >> RHS_CONTROL_PREAMBLE_SIZE_SHIFT) as u8,
        header_size: ((control & RHS_CONTROL_HEADER_SIZE_MASK) >> RHS_CONTROL_HEADER_SIZE_SHIFT)
            as u8,
        max_header_count_cap: ((control & RHS_CONTROL_MAX_HEADER_COUNT_CAP_MASK)
            >> RHS_CONTROL_MAX_HEADER_COUNT_CAP_SHIFT) as u8,
        driver_header_count: ((control & RHS_CONTROL_MAX_HEADER_COUNT_MASK)
            >> RHS_CONTROL_MAX_HEADER_COUNT_SHIFT) as u8,
        ring_headers_offset,
    }
}

pub fn rhs_info(regs: &I3cHciRegisters, sections: HciSections) -> Result<RhsInfo> {
    if sections.ring_headers_offset == 0 {
        return Err(I3cHciError::Unsupported);
    }

    let info = parse_rhs_control(regs.rhs_control(), sections.ring_headers_offset);

    if info.max_header_count_cap == 0 || info.header_size == 0 {
        return Err(I3cHciError::Unsupported);
    }

    Ok(info)
}

pub fn init_rhs(regs: &I3cHciRegisters, sections: HciSections, ring_count: u8) -> Result<RhsInfo> {
    let info = rhs_info(regs, sections)?;

    if ring_count == 0 || ring_count > info.max_header_count_cap || ring_count > 8 {
        return Err(I3cHciError::InvalidArgs);
    }

    let control = (regs.rhs_control() & !RHS_CONTROL_MAX_HEADER_COUNT_MASK)
        | (u32::from(ring_count) << RHS_CONTROL_MAX_HEADER_COUNT_SHIFT);

    regs.write_rhs_control(control);

    Ok(parse_rhs_control(
        regs.rhs_control(),
        sections.ring_headers_offset,
    ))
}
