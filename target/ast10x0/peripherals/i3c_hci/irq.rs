// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Interrupt dispatch for the AST1080 MIPI I3C HCI.
//!
//! Follows the existing OpenPRoT AST10x0 I3C flag-and-defer model:
//!
//! - the ISR owns a separate MMIO register facade;
//! - the ISR never borrows `I3cHciCore`, `I3cHciMaster`, or `I3cHciPio`;
//! - interrupt state is latched into per-bus atomics;
//! - transfer/FIFO processing remains in normal driver context;
//! - the integration layer owns the NVIC vector and mask.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;

use super::constants::{
    ASPEED_INTR_SUM_CAP, ASPEED_INTR_SUM_INHOUSE, ASPEED_INTR_SUM_PIO, ASPEED_INTR_SUM_RHS,
    MAX_BUSES, MIPI_I3C_HCI_CORE_IRQS,
};
use super::core::IoMode;
use super::registers::I3cHciRegisters;

/// Everything needed directly by the ISR for one HCI instance.
///
/// This context intentionally contains no references to thread-owned driver
/// objects.
pub struct IsrCtx {
    regs: I3cHciRegisters,
    io_mode: IoMode,
}

// SAFETY:
// `IsrCtx` contains only the MMIO register facade and a small value type.
// Access to a parked context is serialized by `critical_section`.
unsafe impl Send for IsrCtx {}

impl IsrCtx {
    /// Construct an ISR context for one HCI bus.
    ///
    /// # Safety
    ///
    /// The caller must ensure this register handle refers to the same physical
    /// controller whose normal driver access is serialized against the ISR.
    pub unsafe fn new(bus: u8, io_mode: IoMode) -> Option<Self> {
        let regs = unsafe { I3cHciRegisters::new(bus) }?;

        Some(Self { regs, io_mode })
    }
}

/// State passed from ISR context back to normal driver context.
///
/// The ISR only latches events here. It does not touch transfer buffers or
/// thread-owned driver state.
pub struct IsrEvents {
    core_status: AtomicU32,
    io_status: AtomicU32,
    vendor_status: AtomicU32,
    masked_io_signals: AtomicU32,
}

impl IsrEvents {
    const fn new() -> Self {
        Self {
            core_status: AtomicU32::new(0),
            io_status: AtomicU32::new(0),
            vendor_status: AtomicU32::new(0),
            masked_io_signals: AtomicU32::new(0),
        }
    }

    /// Atomically take the pending HCI core interrupt bits.
    pub fn take_core_status(&self) -> u32 {
        self.core_status.swap(0, Ordering::AcqRel)
    }

    /// Atomically take the pending PIO/DMA interrupt bits.
    pub fn take_io_status(&self) -> u32 {
        self.io_status.swap(0, Ordering::AcqRel)
    }

    /// Serialize task-side sampling with ISR latching/masking. FIFO levels in
    /// the latch are wakeup hints only; callers must use the live level.
    pub(crate) fn snapshot_io_status(&self, read: impl FnOnce() -> u32) -> (u32, u32) {
        critical_section::with(|_| (read(), self.take_io_status()))
    }

    /// The deferred consumer owns W1C. Retire only its sampled bits while the
    /// ISR is excluded, preserving unrelated command/IBI evidence.
    pub(crate) fn ack_io_status(&self, mask: u32, acknowledge: impl FnOnce()) {
        critical_section::with(|_| {
            acknowledge();
            self.io_status.fetch_and(!mask, Ordering::AcqRel);
        });
    }

    pub(crate) fn rearm_io_signals(&self, rearm: impl FnOnce(u32)) {
        critical_section::with(|_| {
            rearm(self.masked_io_signals.swap(0, Ordering::AcqRel));
        });
    }

    /// Peek at pending PIO/DMA interrupt bits without clearing them.
    pub fn pending_io_status(&self) -> u32 {
        self.io_status.load(Ordering::Acquire)
    }

    /// Atomically take pending ASPEED vendor/private summary bits.
    pub fn take_vendor_status(&self) -> u32 {
        self.vendor_status.swap(0, Ordering::AcqRel)
    }

    /// Drop stale software state while NVIC is masked, before registration.
    /// This does not acknowledge hardware or restore masked signal enables.
    /// Never clear an active transport's latch.
    pub fn clear(&self) {
        self.core_status.store(0, Ordering::Release);
        self.io_status.store(0, Ordering::Release);
        self.vendor_status.store(0, Ordering::Release);
        self.masked_io_signals.store(0, Ordering::Release);
    }
}

static ISR_EVENTS: [IsrEvents; MAX_BUSES] = [
    IsrEvents::new(),
    IsrEvents::new(),
    IsrEvents::new(),
    IsrEvents::new(),
    IsrEvents::new(),
    IsrEvents::new(),
    IsrEvents::new(),
    IsrEvents::new(),
];

static BUS_ISR: [Mutex<UnsafeCell<Option<IsrCtx>>>; MAX_BUSES] = [
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
    Mutex::new(UnsafeCell::new(None)),
];

#[must_use]
pub fn isr_events(bus: usize) -> Option<&'static IsrEvents> {
    ISR_EVENTS.get(bus)
}

/// Claim the interrupt-dispatch slot for one HCI bus.
///
/// One live HCI controller is allowed per physical bus.
#[must_use]
pub fn register_i3c_hci_irq_handler(bus: usize, ctx: IsrCtx) -> bool {
    if bus != usize::from(ctx.regs.bus()) {
        return false;
    }
    let Some(slot) = BUS_ISR.get(bus) else {
        return false;
    };

    critical_section::with(|cs| {
        // SAFETY:
        // Access is serialized by the critical section and the reference does
        // not escape this function.
        let parked = unsafe { &mut *slot.borrow(cs).get() };

        if parked.is_some() {
            return false;
        }

        *parked = Some(ctx);

        if let Some(events) = ISR_EVENTS.get(bus) {
            events.clear();
        }

        true
    })
}

/// Release an ISR slot after a failed bring-up.
///
/// Do not call this on a live controller.
pub fn unregister_i3c_hci_irq_handler(bus: usize) {
    let Some(slot) = BUS_ISR.get(bus) else {
        return;
    };

    critical_section::with(|cs| {
        // SAFETY: serialized by the critical section.
        let parked = unsafe { &mut *slot.borrow(cs).get() };
        *parked = None;
    });
}

/// Entry point called by the platform/kernel interrupt vector.
#[inline]
pub fn dispatch_i3c_hci_irq(bus: usize) {
    critical_section::with(|cs| {
        let Some(slot) = BUS_ISR.get(bus) else {
            return;
        };

        // SAFETY:
        // The critical section excludes registration/unregistration while this
        // borrowed context is in use.
        let parked = unsafe { &*slot.borrow(cs).get() };

        if let Some(ctx) = parked {
            isr_service(ctx);
        }
    });
}

fn isr_service(ctx: &IsrCtx) {
    let regs = &ctx.regs;
    let bus = usize::from(regs.bus());

    let Some(events) = ISR_EVENTS.get(bus) else {
        return;
    };

    let mut summary = regs.aspeed_intr_summary();

    /*
     * Match the Zephyr vendor IRQ routing:
     *
     * CAP       -> HCI core
     * PIO/RHS   -> selected I/O backend
     * INHOUSE   -> ASPEED private/vendor events
     */

    if (summary & ASPEED_INTR_SUM_CAP) != 0 {
        handle_core_irq(regs, events);
        summary &= !ASPEED_INTR_SUM_CAP;
    }

    if (summary & (ASPEED_INTR_SUM_PIO | ASPEED_INTR_SUM_RHS)) != 0 {
        match ctx.io_mode {
            IoMode::Pio => handle_pio_irq(regs, events),

            IoMode::Dma => handle_rhs_irq(regs, events),
        }

        summary &= !(ASPEED_INTR_SUM_PIO | ASPEED_INTR_SUM_RHS);
    }

    if (summary & ASPEED_INTR_SUM_INHOUSE) != 0 {
        /*
         * Preserve the fact that a vendor interrupt occurred without touching
         * target/IBI state in the ISR. Normal context can decode it later.
         */
        events
            .vendor_status
            .fetch_or(ASPEED_INTR_SUM_INHOUSE, Ordering::AcqRel);

        summary &= !ASPEED_INTR_SUM_INHOUSE;
    }

    /*
     * AST1080/ASPEED interrupt aggregator acknowledgement.
     */
    regs.aspeed_intr_renew();

    /*
     * Preserve unexpected summary bits for diagnostics instead of silently
     * discarding them.
     */
    if summary != 0 {
        events.vendor_status.fetch_or(summary, Ordering::AcqRel);
    }
}

fn handle_core_irq(regs: &I3cHciRegisters, events: &IsrEvents) {
    let status = regs.intr_status();

    if status == 0 {
        return;
    }

    /*
     * INTR_STATUS is W1C. Latch before clearing so normal driver context can
     * decide whether recovery is required.
     */
    let relevant = status & MIPI_I3C_HCI_CORE_IRQS;

    if relevant != 0 {
        events.core_status.fetch_or(relevant, Ordering::AcqRel);
    }

    regs.write_intr_status(status);
}

fn handle_rhs_irq(regs: &I3cHciRegisters, events: &IsrEvents) {
    let status = regs.dma_ring_intr_status();

    if status == 0 {
        return;
    }

    events.io_status.fetch_or(status, Ordering::AcqRel);
    // Leave W1C to the task: a second acknowledgement here/there could clear
    // a newer same-bit notification. Mask until the consumer acknowledges.
    events
        .masked_io_signals
        .fetch_or(regs.dma_ring_intr_signal_enable(), Ordering::AcqRel);
    regs.write_dma_ring_intr_signal_enable(0);
}
fn handle_pio_irq(regs: &I3cHciRegisters, events: &IsrEvents) {
    let status = regs.pio_intr_status();

    if status == 0 {
        return;
    }

    /*
     * Do NOT drain response/RX/TX FIFOs here.
     *
     * The caller's HciTransfer may contain borrowed buffers, so the OpenPRoT
     * architecture intentionally defers FIFO work to normal driver context.
     */
    events.io_status.fetch_or(status, Ordering::AcqRel);

    // All FIFO work and W1C acknowledgement belong to deferred context.
    // Save the complete policy, including IBI, before masking level sources.
    events
        .masked_io_signals
        .fetch_or(regs.pio_intr_signal_enable(), Ordering::AcqRel);
    regs.write_pio_intr_signal_enable(0);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn acknowledgement_preserves_unrelated_events() {
        let events = IsrEvents::new();
        events
            .io_status
            .store((1 << 11) | (1 << 12), Ordering::Release);
        events.ack_io_status(1 << 11, || {});
        assert_eq!(events.pending_io_status(), 1 << 12);
    }
    #[test]
    fn rearm_takes_saved_subscription_once() {
        let events = IsrEvents::new();
        events.masked_io_signals.store(0x1010, Ordering::Release);
        events.rearm_io_signals(|saved| assert_eq!(saved, 0x1010));
        events.rearm_io_signals(|saved| assert_eq!(saved, 0));
    }
    #[test]
    fn snapshot_keeps_live_levels_separate_from_latched_hints() {
        let events = IsrEvents::new();
        events.io_status.store(0x10, Ordering::Release);
        assert_eq!(events.snapshot_io_status(|| 0), (0, 0x10));
        assert_eq!(events.pending_io_status(), 0);
    }
}
