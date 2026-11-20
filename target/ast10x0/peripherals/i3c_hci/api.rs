// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Public I3C contracts. Transport, descriptors and DAT allocation stay in the driver.
//! Transfers are synchronous; polling can be scheduled by bare-metal or kernel code.
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
    fn read(&mut self, device: I3cDevice, data: &mut [u8]) -> Result<usize>;
    fn ccc_write(&mut self, device: I3cDevice, ccc: u8, data: &[u8]) -> Result<()>;
    fn ccc_read(&mut self, device: I3cDevice, ccc: u8, data: &mut [u8]) -> Result<usize>;
    fn enable_ibi(&mut self, device: I3cDevice, max_payload: u16) -> Result<()>;
    fn disable_ibi(&mut self, device: I3cDevice) -> Result<()>;
    fn poll_event(&mut self, payload: &mut [u8]) -> Result<Option<ControllerEvent>>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetEvent {
    WriteReceived {
        len: usize,
    },
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
    fn queue_read(&mut self, data: &[u8]) -> Result<()>;
    fn queue_ibi(&mut self, payload: &[u8]) -> Result<()>;
    fn start_ibi(&mut self) -> Result<()>;
    fn poll_event(&mut self, data: &mut [u8]) -> Result<Option<TargetEvent>>;
}
