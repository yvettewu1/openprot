// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Controller session: owns discovery state and selects the HCI transport.
use super::api::{ControllerEvent, I3cController, I3cDevice};
use super::core::{I3cHciCore, IoMode};
use super::dat::DatAllocator;
use super::dma::Ring;
use super::error::{I3cHciError, Result};
use super::master::{HciDeviceTable, HciI3cDevice, I3cHciMaster};

impl From<HciI3cDevice> for I3cDevice {
    fn from(device: HciI3cDevice) -> Self {
        Self {
            pid: device.pid,
            dynamic_addr: device.dynamic_addr,
            bcr: device.bcr,
            dcr: device.dcr,
        }
    }
}

/// Owns a fresh controller's discovery table, allocator and selected transport.
/// Keep one session for the bus lifetime; do not independently reset/readdress
/// devices through the low-level API while a session owns their assignments.
pub struct Controller<'a, Y: FnMut(u32), const N: usize> {
    master: I3cHciMaster<'a, Y>,
    devices: HciDeviceTable<N>,
    allocator: DatAllocator,
}

impl<'a, Y: FnMut(u32), const N: usize> Controller<'a, Y, N> {
    /// Attach to an initialized, idle primary before discovery. The core's
    /// command capability and selected transport are applied automatically.
    /// scl_hz selects command timing; board setup must configure matching PHY.
    pub fn new<F: FnMut(u32)>(
        core: &'a mut I3cHciCore<F>,
        scl_hz: u32,
        yield_us: Y,
    ) -> Result<Self> {
        let format = core.command_descriptor().ok_or(I3cHciError::InvalidArgs)?;
        let mode = core.selected_io().ok_or(I3cHciError::InvalidArgs)?;
        if N == 0
            || scl_hz == 0
            || core.sections().dat.entries == 0
            || core.regs().controller_device_addr() & super::constants::MASTER_DYNAMIC_ADDR_VALID
                == 0
        {
            return Err(I3cHciError::InvalidArgs);
        }
        let mut master = I3cHciMaster::new(core.regs(), core.sections(), scl_hz, yield_us)
            .with_command_descriptor(format);
        if mode == IoMode::Dma {
            master = master.with_dma(Ring::init(core).map_err(I3cHciError::Dma)?)?;
        }
        Ok(Self {
            master,
            devices: HciDeviceTable::new(),
            allocator: DatAllocator::new(core.sections().dat.entries.min(128) as u8, true),
        })
    }

    pub fn with_timeout_us(mut self, timeout_us: u32) -> Self {
        self.master = self.master.with_timeout_us(timeout_us);
        self
    }

    /// Connect the pre-registered IRQ event source. Board setup owns NVIC.
    pub fn with_irq_events(mut self, bus: usize) -> Self {
        self.master = self.master.with_irq_events(bus);
        self
    }

    pub fn devices(&self) -> impl Iterator<Item = I3cDevice> + '_ {
        self.devices.iter().map(I3cDevice::from)
    }

    fn resolve(&self, device: I3cDevice) -> Result<HciI3cDevice> {
        self.devices
            .by_dynamic_addr(device.dynamic_addr)
            .filter(|entry| I3cDevice::from(*entry) == device)
            .ok_or(I3cHciError::InvalidArgs)
    }

    /// Explicit shutdown, since stopping DMA can fail and cannot be hidden in Drop.
    pub fn shutdown(mut self) -> Result<()> {
        if let Some(ring) = self.master.dma_ring_mut() {
            ring.stop().map_err(I3cHciError::Dma)?;
        }
        Ok(())
    }
}

impl<Y: FnMut(u32), const N: usize> I3cController for Controller<'_, Y, N> {
    fn discover(&mut self) -> Result<usize> {
        self.master
            .entdaa_new_devices(&mut self.devices, &mut self.allocator)
    }
    fn device_by_pid(&self, pid: u64) -> Option<I3cDevice> {
        self.devices.by_pid(pid).map(I3cDevice::from)
    }
    fn device_by_address(&self, address: u8) -> Option<I3cDevice> {
        self.devices.by_dynamic_addr(address).map(I3cDevice::from)
    }
    fn write(&mut self, device: I3cDevice, data: &[u8]) -> Result<()> {
        let device = self.resolve(device)?;
        self.master
            .private_write(device.dat_index, data)
            .map(|_| ())
    }
    fn read(&mut self, device: I3cDevice, data: &mut [u8]) -> Result<usize> {
        let device = self.resolve(device)?;
        self.master
            .private_read(device.dat_index, data)
            .map(|result| result.rx_len)
    }
    fn ccc_write(&mut self, device: I3cDevice, ccc: u8, data: &[u8]) -> Result<()> {
        // Address-changing CCCs would invalidate the session's table. Keep
        // those in bus-management code until typed operations update it too.
        if ccc < 0x80 || matches!(ccc, 0x86 | 0x87 | 0x88) {
            return Err(I3cHciError::Unsupported);
        }
        let device = self.resolve(device)?;
        self.master
            .ccc_direct_write(device.dat_index, ccc, data)
            .map(|_| ())
    }
    fn ccc_read(&mut self, device: I3cDevice, ccc: u8, data: &mut [u8]) -> Result<usize> {
        if ccc < 0x80 {
            return Err(I3cHciError::InvalidArgs);
        }
        let device = self.resolve(device)?;
        self.master
            .ccc_direct_read(device.dat_index, ccc, None, data)
            .map(|result| result.rx_len)
    }
    fn enable_ibi(&mut self, device: I3cDevice, max_payload: u16) -> Result<()> {
        let device = self.resolve(device)?;
        self.master
            .enable_ibi_with_max_payload(device, max_payload)
            .map(|_| ())
    }
    fn disable_ibi(&mut self, device: I3cDevice) -> Result<()> {
        let device = self.resolve(device)?;
        self.master.disable_ibi(device)
    }
    fn poll_event(&mut self, payload: &mut [u8]) -> Result<Option<ControllerEvent>> {
        self.master.poll_controller_event(payload)
    }
}
