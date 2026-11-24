// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! DMA storage, hardware-stride validation and exclusive per-bus allocation.
use super::super::config::{
    BUSES, DMA_INSTANCE_COUNT, DMA_TOTAL_PAYLOAD_BYTES, dma_index, dma_payload_offset,
};
use super::DmaError;
use super::{DmaResult, ENTRIES, IBI_CHUNK_POOL_SIZE, IBI_CHUNK_SIZE, IBI_STATUS_RING_ENTRIES};
use core::{
    cell::Cell,
    ptr::addr_of_mut,
    sync::atomic::{AtomicU32, Ordering},
};
// PAC exposes all three hardware structure-size fields as u8 byte counts.
// Reserve the entire representable range rounded to a DWORD. Slot indexing
// still uses the actual reported stride, never this static capacity.
const DESCRIPTOR_STRIDE_CAPACITY: usize = (u8::MAX as usize + 3) & !3;
pub(super) const COMMAND_STORAGE_BYTES: usize = ENTRIES * DESCRIPTOR_STRIDE_CAPACITY;
pub(super) const RESPONSE_STORAGE_BYTES: usize = ENTRIES * DESCRIPTOR_STRIDE_CAPACITY;
pub(super) const IBI_STATUS_STORAGE_BYTES: usize =
    IBI_STATUS_RING_ENTRIES * DESCRIPTOR_STRIDE_CAPACITY;
// Match the Zephyr ASPEED I3C vendor DMA hook for non-AST27XX:
// pass the buffer address unchanged. The generic G2 CM4 address translator
// is not called by that I3C hook. Keep cacheability separate from addressing.
const DMA_BUS_OFFSET: u32 = 0;

#[derive(Clone, Copy)]
pub(super) struct PayloadAllocation {
    pub(super) offset: usize,
    pub(super) len: usize,
}

/// First-fit allocation among the at most ENTRIES live command buffers.
pub(super) fn payload_offset(
    live: &[Option<PayloadAllocation>; ENTRIES],
    len: usize,
    capacity: usize,
) -> Option<usize> {
    let mut offset = 0usize;
    loop {
        let end = offset.checked_add(len)?;
        if end > capacity {
            return None;
        }
        let overlap = live
            .iter()
            .flatten()
            .find(|a| a.len != 0 && offset < a.offset + a.len && a.offset < end);
        match overlap {
            Some(a) => offset = a.offset + a.len,
            None => return Some(offset),
        }
    }
}

#[repr(C, align(64))]
pub(super) struct DescriptorMemory {
    pub(super) commands: [u32; COMMAND_STORAGE_BYTES / 4],
    pub(super) responses: [u32; RESPONSE_STORAGE_BYTES / 4],
    pub(super) ibi_status: [u32; IBI_STATUS_STORAGE_BYTES / 4],
}

impl DescriptorMemory {
    const fn empty() -> Self {
        Self {
            commands: [0; COMMAND_STORAGE_BYTES / 4],
            responses: [0; RESPONSE_STORAGE_BYTES / 4],
            ibi_status: [0; IBI_STATUS_STORAGE_BYTES / 4],
        }
    }
}

#[repr(C, align(64))]
pub(super) struct IbiDataMemory([u8; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE]);

#[repr(C, align(64))]
struct PayloadMemory<const N: usize>([u8; N]);

// Reserve descriptor/IBI storage for enabled nodes with DMA reservations. Every bus
// receives a private pool segment and private descriptor ring regardless of role.
const _: () = assert!(
    DMA_INSTANCE_COUNT
        * (core::mem::size_of::<DescriptorMemory>() + core::mem::size_of::<IbiDataMemory>())
        + core::mem::size_of::<PayloadMemory<DMA_TOTAL_PAYLOAD_BYTES>>()
        <= 128 * 1024
);

#[unsafe(link_section = ".dma_nc")]
static mut DESCRIPTORS: [DescriptorMemory; DMA_INSTANCE_COUNT] =
    [const { DescriptorMemory::empty() }; DMA_INSTANCE_COUNT];
#[unsafe(link_section = ".dma_nc")]
static mut IBI_DATA: [IbiDataMemory; DMA_INSTANCE_COUNT] =
    [const { IbiDataMemory([0; IBI_CHUNK_SIZE * IBI_CHUNK_POOL_SIZE]) }; DMA_INSTANCE_COUNT];
#[unsafe(link_section = ".dma_nc")]
static mut PAYLOAD: PayloadMemory<DMA_TOTAL_PAYLOAD_BYTES> =
    PayloadMemory([0; DMA_TOTAL_PAYLOAD_BYTES]);

pub(super) fn validate_descriptor_stride(
    stride: usize,
    minimum: usize,
    entries: usize,
    capacity: usize,
) -> DmaResult<()> {
    if stride < minimum || stride % 4 != 0 {
        return Err(DmaError::Fault("invalid hardware DMA descriptor stride"));
    }
    let required = entries
        .checked_mul(stride)
        .ok_or(DmaError::Fault("DMA descriptor storage size overflow"))?;
    if required > capacity {
        return Err(DmaError::Fault(
            "hardware DMA descriptor stride exceeds backing storage",
        ));
    }
    Ok(())
}

pub(super) fn dma_address(ptr: *const u8, len: usize) -> DmaResult<u32> {
    let start = ptr as usize;
    if start < 0x000a0000 || start.checked_add(len).is_none_or(|end| end > 0x000c0000) {
        return Err(DmaError::Fault("DMA allocation is outside reserved SRAM"));
    }
    (start as u32)
        .checked_add(DMA_BUS_OFFSET)
        .ok_or(DmaError::Fault("DMA address overflow"))
}

// A failed shutdown keeps the reservation: hardware may still access SRAM.
static CLAIMED: AtomicU32 = AtomicU32::new(0);
pub(super) struct DmaMemory {
    pub(super) descriptors: *mut DescriptorMemory,
    pub(super) ibi_data: *mut u8,
    pub(super) payload: *mut u8,
    pub(super) payload_capacity: usize,
    bit: u32,
    quiescent: Cell<bool>,
}
impl DmaMemory {
    pub(super) fn claim(bus: usize) -> DmaResult<Self> {
        let index = dma_index(bus).ok_or(DmaError::Fault(
            "bus has no DMA resources in board configuration",
        ))?;
        let bit = 1u32 << bus;
        if CLAIMED.fetch_or(bit, Ordering::AcqRel) & bit != 0 {
            return Err(DmaError::Fault("DMA bus resources already owned"));
        }
        Ok(Self {
            descriptors: unsafe {
                addr_of_mut!(DESCRIPTORS)
                    .cast::<DescriptorMemory>()
                    .add(index)
            },
            ibi_data: unsafe {
                addr_of_mut!(IBI_DATA)
                    .cast::<IbiDataMemory>()
                    .add(index)
                    .cast()
            },
            payload: unsafe {
                addr_of_mut!(PAYLOAD)
                    .cast::<u8>()
                    .add(dma_payload_offset(bus))
            },
            payload_capacity: BUSES[bus].dma_payload_bytes,
            bit,
            quiescent: Cell::new(true),
        })
    }
    pub(super) fn arm(&self) {
        self.quiescent.set(false);
    }
    pub(super) fn stopped(&self) {
        self.quiescent.set(true);
    }
}
impl Drop for DmaMemory {
    fn drop(&mut self) {
        if self.quiescent.get() {
            CLAIMED.fetch_and(!self.bit, Ordering::AcqRel);
        }
    }
}
