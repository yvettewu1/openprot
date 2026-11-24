// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Board lifecycle for external-target controller tests; protocols use traits.
use ast10x0_i3c_hci::{CommandDescriptor, Controller, I3cBusManagement, I3cHciCore, IoMode};
use cortex_m::peripheral::NVIC;
pub type Result<T> = core::result::Result<T, &'static str>;

/// # Safety
/// Call once at standalone boot with exclusive ownership of SCU and all HCI ports.
pub unsafe fn run(
    io: IoMode,
    irq: bool,
    format: Option<CommandDescriptor>,
    test: impl FnOnce(&mut dyn I3cBusManagement) -> Result<()>,
) -> Result<()> {
    let (bus_id, _) = super::i3c_setup::test_bus(true, io)?;
    let interrupt = super::i3c_setup::bus_interrupt(bus_id);
    NVIC::mask(interrupt);
    unsafe { super::i3c_setup::prepare_and_show(&[bus_id], false) };
    crate::info!("Board controller: HCI{} {:?}", bus_id, io);
    let mut core = unsafe { I3cHciCore::from_board_with_io(bus_id, io, |_| {}) }
        .map_err(|_| "controller core creation failed")?;
    let mut registered = false;
    let outcome = (|| -> Result<()> {
        core.init_configured()
            .map_err(|_| "controller init failed")?;
        core.trace_timing(|args| crate::info!("{}", args));
        if format.is_some_and(|required| core.command_descriptor() != Some(required)) {
            return Err("controller command format unsupported");
        }
        core.disable_irq_signals();
        if irq {
            let ctx = core.isr_ctx().map_err(|_| "ISR context creation failed")?;
            if !ast10x0_i3c_hci::register_i3c_hci_irq_handler(bus_id as usize, ctx) {
                return Err("IRQ registration failed");
            }
            registered = true;
            NVIC::unpend(interrupt);
            core.enable_irq_signals();
        } else if io == IoMode::Dma {
            core.enable_irq_signals(); // DMA polling, NVIC remains masked.
        }
        let bus = Controller::<_, 8>::from_configured_core(&mut core, |_| {})
            .map_err(|_| "controller session creation failed")?;
        let mut bus = if irq {
            bus.with_irq_events(bus_id as usize)
        } else {
            bus
        };
        // Subscribe the fully initialized transport before IRQ delivery starts.
        if irq {
            unsafe { NVIC::unmask(interrupt) };
        }
        let result = test(&mut bus);
        // The ISR must not clear status/rearm queues while shutdown stops DMA.
        // The parked context is removed below after the session borrow ends.
        NVIC::mask(interrupt);
        let stopped = bus.shutdown().map_err(|error| {
            crate::error!("Controller shutdown failed: {:?}", error);
            "controller shutdown failed"
        });
        result?;
        stopped
    })();
    NVIC::mask(interrupt);
    core.disable_irq_signals();
    if registered {
        ast10x0_i3c_hci::unregister_i3c_hci_irq_handler(bus_id as usize);
    }
    if outcome.is_err() {
        super::i3c_setup::show_loopback_failure(core.regs(), io);
    }
    outcome
}
