// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Public I3C contracts. Transport, descriptors and DAT allocation stay in the driver.
//! Public transfers are synchronous polling adapters today. IRQ/task scheduling\n//! can drive the internal operation interface when Pigweed integration is added.
use super::error::Result;

/// Device identity exposed to applications. DAT placement stays in the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I3cDevice {
    pub pid: u64,
    pub dynamic_addr: u8,
    pub bcr: u8,
    pub dcr: u8,
}

/// Payload is written into the caller's buffer, valid through payload_len.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerEvent {
    TargetInterrupt {
        dynamic_addr: u8,
        payload_len: usize,
    },
    HotJoin,
    Unknown {
        address: u8,
        payload_len: usize,
    },
}

/// Small application surface; no DAT indices, ring pointers or descriptors.
/// Discovery adds devices without discarding existing assignments.
pub trait I3cController {
    fn discover(&mut self) -> Result<usize>;
    fn device_by_pid(&self, pid: u64) -> Option<I3cDevice>;
    fn device_by_address(&self, address: u8) -> Option<I3cDevice>;
    fn write(&mut self, device: I3cDevice, data: &[u8]) -> Result<()>;
    /// Returns the received length; an oversized response is an error, not truncation.
    fn read(&mut self, device: I3cDevice, data: &mut [u8]) -> Result<usize>;
    fn ccc_write(&mut self, device: I3cDevice, ccc: u8, data: &[u8]) -> Result<()>;
    /// Returns the received length; an oversized response is an error, not truncation.
    fn ccc_read(&mut self, device: I3cDevice, ccc: u8, data: &mut [u8]) -> Result<usize>;
    fn enable_ibi(&mut self, device: I3cDevice, max_payload: u16) -> Result<()>;
    fn disable_ibi(&mut self, device: I3cDevice) -> Result<()>;
    /// Keep the same payload buffer across None results: a segmented IBI may be in progress.
    fn poll_event(&mut self, payload: &mut [u8]) -> Result<Option<ControllerEvent>>;
    /// Abandon a partially copied PIO IBI. Poll until true before reusing the
    /// receive buffer for a new event. Untouched queued events are preserved;
    /// DMA leaves partial events in SRAM and immediately returns true.
    fn discard_partial_ibi(&mut self) -> Result<bool>;
}

/// Explicit topology operations for firmware that manages bus addresses.
/// Successful address changes update the controller's device table. Callers must
/// use the returned identity; handles containing the previous address are stale.
/// An uncertain address change blocks traffic with RecoveryRequired. Broadcast
/// reset_addresses(None) recovers topology if the transport is still usable;
/// otherwise stop/reinitialize the controller before re-enumeration.
pub trait I3cBusManagement: I3cController {
    /// Broadcast a non-address-changing CCC. RSTDAA/ENTDAA use typed methods.
    fn broadcast_ccc(&mut self, ccc: u8, data: &[u8]) -> Result<()>;
    /// None broadcasts RSTDAA. Some sends legacy direct RSTDAA; currently only
    /// supported on a single-device bus. Discards cached discovery after success.
    fn reset_addresses(&mut self, device: Option<I3cDevice>) -> Result<()>;
    fn set_address(&mut self, device: I3cDevice, address: u8) -> Result<I3cDevice>;
    /// SETDASA for a known, currently unaddressed target. identity contains the
    /// desired dynamic address and previously verified PID/BCR/DCR. The caller
    /// must ensure that static_addr identifies that target on this board.
    fn assign_static_address(&mut self, static_addr: u8, identity: I3cDevice) -> Result<I3cDevice>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetEvent {
    WriteReceived {
        len: usize,
    },
    /// Hardware-handled CCC notification. len is received payload bytes;
    /// automatic CCC read replies report zero and do not populate the RX buffer.
    CccReceived {
        ccc: u8,
        len: usize,
    },
    /// Hardware TX completion field; not the controller's received byte count.
    ReadComplete {
        remaining: usize,
    },
    IbiComplete,
    Other,
}

/// Queueing copies data into the driver/hardware; callers can reuse buffers
/// after return. Queue read data and IBI separately so applications control
/// ordering. start_ibi returns after requesting transmission, not completion.
pub trait I3cTarget {
    fn address(&self) -> Option<u8>;
    /// Current hardware SIR state established by ENEC/DISEC.
    fn ibi_enabled(&self) -> bool;
    fn queue_read(&mut self, data: &[u8]) -> Result<()>;
    fn queue_ibi(&mut self, payload: &[u8]) -> Result<()>;
    fn start_ibi(&mut self) -> Result<()>;
    fn poll_event(&mut self, data: &mut [u8]) -> Result<Option<TargetEvent>>;
}
