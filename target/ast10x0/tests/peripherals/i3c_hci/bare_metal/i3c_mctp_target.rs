// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Shared external-controller MCTP target test over PIO or DMA.
//! Applications use I3cTarget for RX, queued replies, IBI, and completion events.

use ast10x0_i3c_hci::{I3cHciCore, I3cHciTarget, I3cTarget, IoMode, TargetEvent};

pub type Result<T> = core::result::Result<T, &'static str>;
const ROUNDS: u8 = 6; // Same coverage in both modes; wraps the DMA command ring.
const COMPLETION_POLLS: u32 = 10_000_000; // Iterations, not microseconds.
const RX_CAPACITY: usize = 128;

fn poll(bus: &mut impl I3cTarget, rx: &mut [u8], mode: &str) -> Result<Option<TargetEvent>> {
    bus.poll_event(rx).map_err(|error| {
        crate::error!("Target {} event failed: {:?}", mode, error);
        "target event failed"
    })
}

fn capture_request(rx: &[u8], len: usize, pending: &mut [u8]) -> Result<usize> {
    let data = rx.get(..len).ok_or("target RX length exceeds buffer")?;
    if data.is_empty() {
        return Err("empty packet cannot be echoed");
    }
    let destination = pending
        .get_mut(..len)
        .ok_or("target echo exceeds capacity")?;
    destination.copy_from_slice(data);
    crate::info!("MCTP RX len={} data={:02x?}", len, data);
    Ok(len)
}

fn exchange(bus: &mut impl I3cTarget, mode: &str) -> Result<()> {
    // Linux may negotiate MRL/MWL and query identity/status before private traffic.
    let mut rx = [0u8; RX_CAPACITY];
    let mut pending = [0u8; RX_CAPACITY];
    crate::info!("READY: start AST2700 discovery, then mctp-i3c -w <data> -r -c 6 (echo)");
    let mut request_len = None;
    for round in 0..ROUNDS {
        // Operator/discovery waits have no deadline. Hardware handles CCCs; keep
        // draining their RX/CCC records so they cannot block the private write.
        while request_len.is_none() {
            match poll(bus, &mut rx, mode)? {
                Some(TargetEvent::WriteReceived { len }) => {
                    request_len = Some(capture_request(&rx, len, &mut pending)?);
                }
                Some(TargetEvent::CccReceived { .. }) | Some(TargetEvent::Other) | None => {}
                _ => return Err("unexpected completion before request"),
            }
            core::hint::spin_loop();
        }
        let len = request_len.take().ok_or("target echo request missing")?;
        let data = &pending[..len];
        let address = bus.address().ok_or("target has no dynamic address")?;
        let ibi = crate::i3c_mctp_setup::ibi_payload(address, 0xae);
        crate::info!("MCTP TX len={} data={:02x?}", len, data);
        // FIFO order matters: Linux's target pending-read flow queues IBI first,
        // then read data, then raises IBI. Both descriptors must be ready before
        // the external controller reacts; queueing read first can block the IBI.
        bus.queue_ibi(&ibi).map_err(|error| {
            crate::error!("{} IBI queue failed: {:?}", mode, error);
            "target IBI queue failed"
        })?;
        bus.queue_read(data).map_err(|error| {
            crate::error!("{} read-data queue failed: {:?}", mode, error);
            "target read-data queue failed"
        })?;
        bus.start_ibi().map_err(|error| {
            crate::error!("IBI start failed (master must enable SIR): {:?}", error);
            "IBI start failed"
        })?;

        let mut ibi_done = false;
        let mut read_done = false;
        for _ in 0..COMPLETION_POLLS {
            match poll(bus, &mut rx, mode)? {
                Some(TargetEvent::IbiComplete) if !ibi_done => ibi_done = true,
                Some(TargetEvent::ReadComplete { remaining: 0 }) if !read_done => {
                    read_done = true;
                }
                Some(TargetEvent::ReadComplete { .. }) => {
                    return Err("target read incomplete or duplicate completion");
                }
                // RX records have priority over TX completions in the target API.
                // The master may already have read the response and sent its next
                // request before we consume ReadComplete. Retain its bytes separately
                // from RX scratch so later CCC notifications cannot corrupt the echo.
                Some(TargetEvent::WriteReceived { len })
                    if request_len.is_none() && round + 1 < ROUNDS =>
                {
                    request_len = Some(capture_request(&rx, len, &mut pending)?);
                }
                // Queries are harmless; a bus reset/reconfiguration in the middle
                // of a pending response is not a successful exchange.
                Some(TargetEvent::CccReceived {
                    ccc: 0x8b..=0x90 | 0x94,
                    ..
                }) => {}
                Some(TargetEvent::Other) | None => {}
                _ => return Err("unexpected event while response pending"),
            }
            if ibi_done && read_done {
                break;
            }
            core::hint::spin_loop();
        }
        if !ibi_done {
            return Err("target IBI completion timeout");
        }
        if !read_done {
            return Err("AST2700 private read completion timeout");
        }
        crate::info!(
            "ROUND {} PASS: {} RX received, IBI={:02x?} DA={:02x}, {} TX completed",
            round,
            mode,
            ibi,
            address,
            mode
        );
    }
    Ok(())
}

/// Run six write/IBI/read exchanges with an external AST2700 controller.
///
/// # Safety
/// Call once at standalone boot with exclusive ownership of SCU, HCI ports,
/// and their DMA storage. The caller must not run another bus session.
pub unsafe fn run(io: IoMode) -> Result<()> {
    let mode = mode_name(io);
    let (bus_id, config) = crate::i3c_setup::test_bus(false, io)?;
    let identity = config.role.identity().ok_or("target identity missing")?;
    if identity.dcr != 0xcc || identity.bcr & 6 != 6 {
        return Err("board target must advertise MCTP DCR=cc and IBI payload support");
    }
    cortex_m::peripheral::NVIC::mask(crate::i3c_setup::bus_interrupt(bus_id));
    // SAFETY: standalone test owns SCU, all HCI instances and their DMA storage.
    unsafe { crate::i3c_setup::prepare_loopback(&[bus_id], false) };
    crate::info!(
        "MCTP {} TARGET: HCI{} PID={:012x}; external AST2700 is controller",
        mode,
        bus_id,
        identity.pid
    );
    // SAFETY: this test exclusively owns the board's dedicated target port.
    let mut core = unsafe { I3cHciCore::from_board_with_io(bus_id, io, |_| {}) }
        .map_err(|_| "target core creation failed")?;
    let outcome = (|| -> Result<()> {
        core.init_configured()
            .map_err(|_| "target core init failed")?;
        core.trace_timing(|args| crate::info!("{}", args));
        core.enable_irq_signals(); // Polling owns completions; NVIC stays masked.
        let mut target =
            I3cHciTarget::from_configured_core(&mut core, |_| {}).map_err(|error| {
                crate::error!("Target {} setup failed: {:?}", mode, error);
                "target setup failed"
            })?;
        let result = exchange(&mut target, mode);
        // Always stop, including failure after queueing just one descriptor.
        // A failed stop retains the DMA memory claim in the driver.
        let stopped = target.shutdown().map_err(|error| {
            crate::error!("Target {} shutdown failed: {:?}", mode, error);
            if io == IoMode::Dma {
                crate::error!("DMA memory remains reserved");
            }
            "target shutdown failed"
        });
        result?;
        stopped
    })();
    if outcome.is_err() {
        crate::i3c_setup::show_loopback_failure(core.regs(), io);
    }
    core.disable_irq_signals();
    outcome
}

fn mode_name(io: IoMode) -> &'static str {
    match io {
        IoMode::Pio => "PIO",
        IoMode::Dma => "DMA",
    }
}

/// Report the result only after run() has completed target shutdown.
pub fn finish(result: Result<()>, io: IoMode) -> ! {
    match result {
        Ok(()) => crate::write_all(b"TEST_RESULT:PASS\n"),
        Err(error) => {
            crate::error!("MCTP {} target test failed: {}", mode_name(io), error);
            crate::write_all(b"TEST_RESULT:FAIL\n");
        }
    }
    loop {
        core::hint::spin_loop();
    }
}
