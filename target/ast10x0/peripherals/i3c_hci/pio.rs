// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Minimal polling PIO master path for MIPI I3C HCI.
//!
//! This follows the command/data/response ordering in Zephyr pio.c: queue data,
//! queue response expectation, write command descriptor words, then process the
//! response and trailing RX in polling mode.

use super::cmd::{HciCommand, HciCommandKind, HciResponse, HciTargetCommand};
use super::error::{I3cHciError, Result};
use super::irq::{IsrEvents, isr_events};
use super::master::{HciTransfer, HciTransferResult, validate_transfer};
use super::registers::I3cHciRegisters;

pub const STAT_TRANSFER_BLOCKED: u32 = 1 << 25;
pub const STAT_PERR_RESP_UFLOW: u32 = 1 << 24;
pub const STAT_PERR_CMD_OFLOW: u32 = 1 << 23;
pub const STAT_PERR_IBI_UFLOW: u32 = 1 << 22;
pub const STAT_PERR_RX_UFLOW: u32 = 1 << 21;
pub const STAT_PERR_TX_OFLOW: u32 = 1 << 20;
pub const STAT_ERR_RESP_QUEUE_FULL: u32 = 1 << 19;
pub const STAT_ERR_IBI_QUEUE_FULL: u32 = 1 << 17;
pub const STAT_ERR_RX_DATA_FULL: u32 = 1 << 15;
pub const STAT_ERR_TX_DATA_EMPTY: u32 = 1 << 13;
pub const STAT_TRANSFER_ERR: u32 = 1 << 9;
pub const STAT_TRANSFER_ABORT: u32 = 1 << 5;
pub const STAT_RESP_READY: u32 = 1 << 4;
pub const STAT_CMD_QUEUE_READY: u32 = 1 << 3;
pub const STAT_IBI_STATUS_THLD: u32 = 1 << 2;
pub const STAT_TX_THLD: u32 = 1 << 0;

pub const STAT_PROG_ERRORS: u32 = STAT_TRANSFER_BLOCKED
    | STAT_PERR_RESP_UFLOW
    | STAT_PERR_CMD_OFLOW
    | STAT_PERR_IBI_UFLOW
    | STAT_PERR_RX_UFLOW
    | STAT_PERR_TX_OFLOW;
pub const STAT_LATENCY_ERRORS: u32 = STAT_ERR_RESP_QUEUE_FULL
    | STAT_ERR_IBI_QUEUE_FULL
    | STAT_ERR_RX_DATA_FULL
    | STAT_ERR_TX_DATA_EMPTY;
pub const STAT_ALL_ERRORS: u32 =
    STAT_TRANSFER_ABORT | STAT_TRANSFER_ERR | STAT_LATENCY_ERRORS | STAT_PROG_ERRORS;

pub struct PioTransferPairResult {
    pub first_response: HciResponse,
    pub second_response: HciResponse,
    pub rx_len: usize,
}

pub use super::cmd::IbiStatus;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TargetResponse {
    pub raw: u32,
    pub status: u8,

    /// true means the target received data:
    /// controller -> target private write.
    pub target_received: bool,

    pub ccc: bool,
    pub tid: u8,
    pub ccc_hdr: u8,
    pub data_len: usize,
}

impl TargetResponse {
    #[must_use]
    pub const fn parse(raw: u32) -> Self {
        Self {
            raw,
            status: ((raw >> 28) & 0x0f) as u8,
            target_received: ((raw >> 27) & 1) != 0,
            ccc: ((raw >> 26) & 1) != 0,
            tid: ((raw >> 24) & 0x03) as u8,
            ccc_hdr: ((raw >> 16) & 0xff) as u8,
            data_len: (raw & 0xffff) as usize,
        }
    }

    /// Bytes deposited in the target RX FIFO/DMA chunk pool. A target-transmit
    /// notification (including an automatic CCC read) owns no RX data, even
    /// when its response length field is nonzero.
    #[must_use]
    pub const fn received_len(self) -> usize {
        if self.target_received {
            self.data_len
        } else {
            0
        }
    }

    #[must_use]
    pub const fn success(self) -> bool {
        self.status == 0
    }
}

/// State of a segmented IBI. No payload allocation: callers retain the same
/// output buffer until the event completes (or the controller is reset).
#[derive(Default)]
struct IbiAssembly {
    total: usize,
    source: Option<(u8, bool)>,
    failure: Option<I3cHciError>,
    buffer: usize,
    capacity: usize,
    discard: bool,
}

pub struct I3cHciPio<'a, Y: FnMut(u32)> {
    regs: &'a I3cHciRegisters,
    ibi: Option<IbiAssembly>,
    ibi_faulted: bool,
    command_faulted: bool,
    yield_us: Y,
    poll_budget: u32,
    irq_events: Option<&'static IsrEvents>,
    pending_irq_status: u32,
    trace: Option<fn(core::fmt::Arguments<'_>)>,
}

impl<'a, Y: FnMut(u32)> I3cHciPio<'a, Y> {
    #[must_use]
    pub const fn new(regs: &'a I3cHciRegisters, yield_us: Y) -> Self {
        Self {
            regs,
            ibi: None,
            ibi_faulted: false,
            command_faulted: false,
            yield_us,
            poll_budget: 10_000,
            irq_events: None,
            pending_irq_status: 0,
            trace: None,
        }
    }

    #[must_use]
    pub const fn with_poll_budget(mut self, poll_budget: u32) -> Self {
        self.poll_budget = poll_budget;
        self
    }

    #[must_use]
    pub fn with_irq_events(mut self, bus: usize) -> Self {
        if bus == self.regs.bus() as usize {
            self.irq_events = isr_events(bus);
            if let Some(events) = self.irq_events {
                self.pending_irq_status |= events.take_io_status() & STAT_ALL_ERRORS;
            }
        }
        self
    }

    /// Optional synchronous diagnostics. The callback must not access this controller.
    /// Logging adds latency; no callback is installed by default.
    #[must_use]
    pub fn with_trace(mut self, trace: fn(core::fmt::Arguments<'_>)) -> Self {
        self.trace = Some(trace);
        self
    }

    pub fn submit(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        let result = self.submit_raw(transfer)?;
        if !result.response.success() {
            return Err(I3cHciError::Transfer(result.response.status));
        }
        Ok(result)
    }

    pub fn submit_raw(&mut self, transfer: HciTransfer<'_>) -> Result<HciTransferResult> {
        let budget = self.poll_budget.saturating_add(1);
        let operation = self.begin_command(transfer)?;
        super::completion::Polling { budget }.run(operation, I3cHciError::Timeout)
    }

    pub(crate) fn begin_command<'operation, 'buffer>(
        &'operation mut self,
        transfer: HciTransfer<'buffer>,
    ) -> Result<PioCommandOperation<'operation, 'a, 'buffer, Y>> {
        self.require_command_ready()?;
        validate_transfer(&transfer)?;
        if !transfer.command.expects_response() {
            return Err(I3cHciError::Unsupported);
        }
        self.check_errors()?;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "HCI{} submit TID={} TX={} RX={}",
                self.regs.bus(),
                transfer.command.tid(),
                transfer.tx.len(),
                transfer.rx.len()
            ));
            for (index, word) in transfer.command.words().iter().enumerate() {
                trace(format_args!("CMD[{}]={:08x}", index, word));
            }
        }
        let pending = PioCommand {
            transfer,
            tx_offset: 0,
            phase: PioPhase::Tx,
        };
        Ok(PioCommandOperation {
            pio: self,
            pending,
            completed: false,
        })
    }

    /// One bounded hardware step. The descriptor's words remain contiguous.
    fn progress_command(
        &mut self,
        pending: &mut PioCommand<'_>,
    ) -> Result<super::completion::Progress<HciTransferResult>> {
        use super::completion::Progress;
        match pending.phase {
            PioPhase::Tx => {
                if pending.tx_offset == pending.transfer.tx.len() {
                    pending.phase = PioPhase::Command;
                    return Ok(Progress::Advanced);
                }
                if !self.try_status(STAT_TX_THLD)? {
                    return Ok(Progress::Pending);
                }
                let end = (pending.tx_offset + 4).min(pending.transfer.tx.len());
                self.regs.write_pio_tx_data_port(word_from_bytes(
                    &pending.transfer.tx[pending.tx_offset..end],
                ));
                pending.tx_offset = end;
                Ok(Progress::Advanced)
            }
            PioPhase::Command => {
                if !self.try_status(STAT_CMD_QUEUE_READY)? {
                    return Ok(Progress::Pending);
                }
                self.write_command_words(pending.transfer.command.words());
                pending.phase = PioPhase::Response;
                Ok(Progress::Advanced)
            }
            PioPhase::Response => {
                if !self.try_status(STAT_RESP_READY)? {
                    return Ok(Progress::Pending);
                }
                let response = self.consume_response(pending.transfer.command.tid())?;
                // V1 DAA reports assignment count; writes report untransferred
                // TX length. Neither authorizes an RX FIFO read. V2 paired DAA
                // handles its PID payload in submit_pair_raw().
                let received = if pending.transfer.command.read()
                    && pending.transfer.command.kind() != HciCommandKind::AddressAssignment
                {
                    response.data_len
                } else {
                    0
                };
                let rx_len = self.read_rx_data(pending.transfer.rx, received)?;
                self.rearm_pio_irqs(STAT_RESP_READY | STAT_ALL_ERRORS);
                Ok(Progress::Complete(HciTransferResult { response, rx_len }))
            }
        }
    }

    pub fn submit_no_response(&mut self, transfer: HciTransfer<'_>) -> Result<()> {
        self.require_command_ready()?;
        validate_transfer(&transfer)?;
        if transfer.command.expects_response() {
            return Err(I3cHciError::InvalidArgs);
        }
        self.check_errors()?;
        self.command_sequence(|pio| {
            pio.write_tx_data(transfer.tx)?;
            pio.queue_command_words(transfer.command.words())?;
            pio.rearm_pio_irqs(STAT_RESP_READY | STAT_ALL_ERRORS);
            Ok(())
        })
    }

    pub fn submit_pair_raw(
        &mut self,
        first: HciCommand,
        second: HciCommand,
        rx: &mut [u8],
    ) -> Result<PioTransferPairResult> {
        self.require_command_ready()?;
        if !first.expects_response() || !second.expects_response() {
            return Err(I3cHciError::InvalidArgs);
        }
        if !first.read() && first.data_len() != 0 {
            return Err(I3cHciError::InvalidArgs);
        }
        if second.read() || second.data_len() != 0 {
            return Err(I3cHciError::InvalidArgs);
        }

        self.check_errors()?;
        self.command_sequence(|pio| {
            pio.queue_command_words(first.words())?;
            pio.queue_command_words(second.words())?;

            let first_response = pio.wait_response(first.tid())?;
            let rx_len = pio.read_rx_data(rx, first_response.data_len)?;
            let second_response = pio.wait_response(second.tid())?;
            pio.rearm_pio_irqs(STAT_RESP_READY | STAT_ALL_ERRORS);

            Ok(PioTransferPairResult {
                first_response,
                second_response,
                rx_len,
            })
        })
    }

    pub fn target_next_response(&mut self) -> Result<TargetResponse> {
        self.wait_status(STAT_RESP_READY)?;
        self.target_consume_response()
    }

    pub(crate) fn target_poll_response(&mut self) -> Result<Option<TargetResponse>> {
        if !self.try_status(STAT_RESP_READY)? {
            return Ok(None);
        }
        self.target_consume_response().map(Some)
    }

    /// Caller has established RESP_READY; consume exactly one response.
    pub(crate) fn target_consume_response(&mut self) -> Result<TargetResponse> {
        let raw = self.regs.pio_response_queue_port();
        let response = TargetResponse::parse(raw);
        // Log before validation: target errors otherwise disappear before the
        // loopback can explain a short controller read or a halted target.
        if let Some(trace) = self.trace {
            trace(format_args!(
                "TARGET HCI{} response={:08x} status={} tid={} received={} ccc={} len={} HC={:08x} PIO={:08x}",
                self.regs.bus(),
                raw,
                response.status,
                response.tid,
                response.target_received,
                response.ccc,
                response.data_len,
                self.regs.hc_control(),
                self.regs.pio_intr_status(),
            ));
        }

        let status = self.event_status();
        if !response.success() {
            self.ack_errors(STAT_TRANSFER_ERR);
            self.check_status_errors(status & !STAT_TRANSFER_ERR)?;
            if let Some(trace) = self.trace {
                self.regs
                    .trace_transfer_state("target completion error", trace);
            }
            return Err(I3cHciError::Transfer(response.status));
        }
        self.check_status_errors(status)?;
        Ok(response)
    }

    pub fn target_read_rx(&mut self, buf: &mut [u8], count: usize) -> Result<usize> {
        // A consumed response authorizes every word, including the tail.
        // Do not wait for RX_THLD; drain even when the output is too small.
        drain_fifo(buf, count, || Ok(self.regs.pio_rx_data_port()))
    }

    pub fn target_queue_read_data(&mut self, data: &[u8]) -> Result<()> {
        let command = HciTargetCommand::read_data(data.len())?;
        self.target_queue_command(command, data)
    }

    pub fn target_queue_ibi_payload(&mut self, data: &[u8]) -> Result<()> {
        // Match ASPEED Linux prep_ibi_xfer: encode MDB in the descriptor,
        // and put only the remaining bytes into the target TX FIFO.
        let (command, payload) = match data.split_first() {
            Some((&mdb, payload)) => (HciTargetCommand::ibi_with_mdb(mdb, payload.len())?, payload),
            None => (HciTargetCommand::ibi_payload(0)?, data),
        };
        self.target_queue_command(command, payload)
    }

    /// Receive one complete IBI, returning the final descriptor and total bytes.
    /// A descriptor's data_len describes only its segment, not the whole IBI.
    /// Oversized/error transactions are drained through LAST_STATUS before failing.
    /// Blocking compatibility adapter over the same nonblocking receive path.
    pub fn next_ibi(&mut self, payload: &mut [u8]) -> Result<(IbiStatus, usize)> {
        use super::completion::Progress;
        if self.ibi.as_ref().is_some_and(|state| state.discard) {
            return Err(I3cHciError::Busy);
        }
        // Bound malformed streams while giving each segment its own wait budget.
        let mut segments_left = self.poll_budget.max(1);
        super::completion::drive(
            self.poll_budget.max(1),
            || {
                if segments_left == 0 {
                    return Err(I3cHciError::Timeout);
                }
                let progress = self.poll_ibi_progress(payload)?;
                match progress {
                    Progress::Pending => (self.yield_us)(1),
                    Progress::Advanced => segments_left -= 1,
                    Progress::Complete(_) => {}
                }
                Ok(progress)
            },
            I3cHciError::Timeout,
        )
    }

    /// Consume at most one available status and its authorized data. Never wait
    /// for another segment. Keep the same buffer across calls returning None.
    pub(crate) fn poll_ibi(&mut self, payload: &mut [u8]) -> Result<Option<(IbiStatus, usize)>> {
        use super::completion::Progress;
        let discarding = self.ibi.as_ref().is_some_and(|state| state.discard);
        Ok(match self.poll_ibi_progress(payload)? {
            Progress::Complete(event) if !discarding => Some(event),
            _ => None,
        })
    }

    /// Abandon only a partially consumed notification. Each call drains at most
    /// one available segment; no FIFO reset and no wait for another status.
    /// True means no partial event remains. A queued, untouched event is retained.
    pub fn discard_partial_ibi(&mut self) -> Result<bool> {
        if self.ibi_faulted {
            return Err(I3cHciError::RecoveryRequired);
        }
        let Some(state) = &mut self.ibi else {
            return Ok(true);
        };
        state.discard = true;
        // Buffer overflow is irrelevant once the caller abandons its payload.
        if state.failure == Some(I3cHciError::NoSpace) {
            state.failure = None;
        }
        self.poll_ibi_progress(&mut [])?;
        Ok(self.ibi.is_none())
    }

    fn poll_ibi_progress(
        &mut self,
        payload: &mut [u8],
    ) -> Result<super::completion::Progress<(IbiStatus, usize)>> {
        use super::completion::Progress;
        if self.ibi_faulted {
            return Err(I3cHciError::RecoveryRequired);
        }
        let buffer = payload.as_mut_ptr() as usize;
        if let Some(state) = &self.ibi {
            if !state.discard && (state.buffer != buffer || state.capacity != payload.len()) {
                return Err(I3cHciError::InvalidArgs);
            }
        }
        if !self.try_status(STAT_IBI_STATUS_THLD)? {
            return Ok(Progress::Pending);
        }
        let status = self.consume_ibi_status()?;
        let mut state = self.ibi.take().unwrap_or(IbiAssembly {
            buffer,
            capacity: payload.len(),
            ..IbiAssembly::default()
        });
        let identity = (status.target_addr, status.read);
        if state.source.is_some_and(|previous| previous != identity) {
            state.failure = Some(I3cHciError::TransferStatus(status.raw));
        }
        state.source = Some(identity);
        let offset = state.total.min(payload.len());
        let output = if state.discard {
            &mut [][..]
        } else {
            &mut payload[offset..]
        };
        if let Err(error) = self.read_ibi_data(output, status.data_len) {
            if error != I3cHciError::NoSpace {
                // Status is already popped, but the remaining payload position
                // is uncertain. Never parse a payload word as a new status.
                self.ibi = Some(state);
                self.ibi_faulted = true;
                return Err(error);
            }
            if !state.discard {
                state.failure.get_or_insert(error);
            }
        }
        state.total = state.total.saturating_add(status.data_len);
        if status.error {
            state.failure = Some(I3cHciError::TransferStatus(status.raw));
        }
        // The ISR masks level-triggered PIO signals while FIFO work is deferred.
        // Rearm after the entire segment is drained, including intermediate ones.
        self.rearm_pio_irqs(STAT_IBI_STATUS_THLD | STAT_ALL_ERRORS);
        if status.last || !status.read {
            if let Some(error) = state.failure {
                return Err(error);
            }
            return Ok(Progress::Complete((status, state.total)));
        }
        self.ibi = Some(state);
        Ok(Progress::Advanced)
    }

    fn read_ibi_status(&mut self) -> Result<IbiStatus> {
        self.wait_status(STAT_IBI_STATUS_THLD)?;
        self.consume_ibi_status()
    }

    fn consume_ibi_status(&mut self) -> Result<IbiStatus> {
        let raw = self.regs.pio_ibi_port();
        if let Some(trace) = self.trace {
            trace(format_args!("IBI raw status = 0x{:08x}", raw));
        }

        let status = IbiStatus::parse(raw);
        if let Some(trace) = self.trace {
            trace(format_args!(
                "IBI len={} addr=0x{:02x} last={} error={}",
                status.data_len, status.target_addr, status.last, status.error
            ));
        }
        // FIFO readiness is checked live. Preserve only deferred error evidence.
        if let Some(events) = self.irq_events {
            self.pending_irq_status |= events.take_io_status() & STAT_ALL_ERRORS;
        }
        Ok(status)
    }

    /// Low-level single-segment API. Use next_ibi for completed notifications.
    pub fn next_ibi_status(&mut self, payload: &mut [u8]) -> Result<IbiStatus> {
        if self.ibi_faulted {
            return Err(I3cHciError::RecoveryRequired);
        }
        if self.ibi.is_some() {
            return Err(I3cHciError::Busy);
        }
        let status = self.read_ibi_status()?;
        let read = self.read_ibi_data(payload, status.data_len);
        if read
            .as_ref()
            .is_err_and(|error| *error != I3cHciError::NoSpace)
        {
            self.ibi_faulted = true;
        } else {
            // Even a short output buffer drains the full segment. Restore
            // notification delivery after returning its FIFO ownership.
            self.rearm_pio_irqs(STAT_IBI_STATUS_THLD | STAT_ALL_ERRORS);
        }

        if status.error {
            return Err(I3cHciError::TransferStatus(status.raw));
        }

        read.map(|_| status)
    }

    pub(crate) fn target_queue_command(
        &mut self,
        command: HciTargetCommand,
        data: &[u8],
    ) -> Result<()> {
        if command.data_len != data.len() {
            return Err(I3cHciError::InvalidArgs);
        }
        self.check_errors()?;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "TARGET HCI{} queue cmd={:08x} tid={} bytes={} payload={:02x?} HC={:08x} PIO={:08x}",
                self.regs.bus(),
                command.word,
                command.tid,
                data.len(),
                data,
                self.regs.hc_control(),
                self.regs.pio_intr_status()
            ));
        }

        /*
         * Zephyr queues target TX/IBI data before putting the target
         * descriptor into the command queue.
         */
        if let Some(trace) = self.trace {
            self.regs
                .trace_transfer_state("before target TX fill", trace);
        }
        self.write_tx_data(data)?;
        if let Some(trace) = self.trace {
            self.regs
                .trace_transfer_state("after target TX fill", trace);
        }

        self.wait_status(STAT_CMD_QUEUE_READY)?;

        /*
         * Target command descriptors are one word.
         */
        self.regs.write_pio_command_queue_port(command.word);
        // Publish the queued target transfer before the other local HCI is
        // allowed to initiate its controller read.
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        if let Some(trace) = self.trace {
            trace(format_args!(
                "TARGET HCI{} queued HC={:08x} PIO={:08x}",
                self.regs.bus(),
                self.regs.hc_control(),
                self.regs.pio_intr_status()
            ));
            self.regs
                .trace_transfer_state("after target command", trace);
        }

        Ok(())
    }

    pub fn wait_aspeed_cap_ctrl_clear(&mut self, mask: u32) -> Result<()> {
        self.wait_register_clear(
            I3cHciRegisters::aspeed_slave_cap_ctrl,
            mask,
            self.poll_budget,
        )
    }

    fn write_command_words(&self, words: &[u32]) {
        for &word in words {
            self.regs.write_pio_command_queue_port(word);
        }
    }

    fn queue_command_words(&mut self, words: &[u32]) -> Result<()> {
        self.wait_status(STAT_CMD_QUEUE_READY)?;
        self.write_command_words(words);
        Ok(())
    }

    fn write_tx_data(&mut self, tx: &[u8]) -> Result<()> {
        let mut offset = 0;
        while offset < tx.len() {
            self.wait_status(STAT_TX_THLD)?;
            let chunk = core::cmp::min(4, tx.len() - offset);
            self.regs
                .write_pio_tx_data_port(word_from_bytes(&tx[offset..offset + chunk]));
            offset += chunk;
        }
        Ok(())
    }

    fn read_rx_data(&mut self, rx: &mut [u8], expected: usize) -> Result<usize> {
        // Drain the entire response on overflow, so the next transfer cannot
        // consume stale words. drain_fifo copies only the caller's capacity.
        let mut offset = 0;
        // Completion authorizes the short tail even when RX_THLD is clear.
        drain_fifo(rx, expected, || {
            let word = self.regs.pio_rx_data_port();
            let chunk = (expected - offset).min(4);
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "RX offset={} bytes={} raw=0x{:08x} PIO_STATUS=0x{:08x}",
                    offset,
                    chunk,
                    word,
                    self.regs.pio_intr_status()
                ));
            }
            offset += chunk;
            Ok(word)
        })
    }

    fn read_ibi_data(&mut self, buf: &mut [u8], count: usize) -> Result<usize> {
        // STATUS_THLD counts status entries, not payload words. The popped
        // descriptor authorizes draining its complete payload without a wait.
        let mut remaining = count;
        let copied = drain_fifo(buf, count, || {
            self.check_errors()?;
            let word = self.regs.pio_ibi_port();
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "IBI remaining={}, IBI FIFO raw word={:#010x}",
                    remaining, word
                ));
            }
            remaining -= remaining.min(4);
            Ok(word)
        });
        // A hardware read failure stops immediately. Overflow is reported only
        // after the final status check, once the full payload has been drained.
        if let Err(error) = &copied {
            if !matches!(error, I3cHciError::NoSpace) {
                return copied;
            }
        }
        self.check_errors()?;
        copied
    }

    /// Recover after a consumed end-of-DAA NACK, with no other commands queued.
    /// ASPEED Zephyr hci_pio_err resets command/data/response queues, then resumes.
    pub(crate) fn recover_after_daa_nack(&mut self) -> Result<()> {
        use super::constants;
        let mask = constants::CMD_QUEUE_RST
            | constants::RESP_QUEUE_RST
            | constants::TX_FIFO_RST
            | constants::RX_FIFO_RST;
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DAA end recovery: HC_CONTROL=0x{:08x} HC_STATUS=0x{:08x} PIO_STATUS=0x{:08x}",
                self.regs.hc_control(),
                self.regs.intr_status(),
                self.regs.pio_intr_status()
            ));
        }
        self.regs.write_reset_control(mask);
        self.wait_register_clear(
            I3cHciRegisters::reset_control,
            mask,
            constants::MIPI_I3C_HCI_RESET_TIMEOUT_US,
        )?;
        self.regs.set_hc_control(constants::HC_CONTROL_RESUME);
        if let Some(trace) = self.trace {
            trace(format_args!(
                "DAA end recovery: queues reset; RESUME requested; HC_CONTROL=0x{:08x}",
                self.regs.hc_control()
            ));
        }
        Ok(())
    }

    fn wait_response(&mut self, tid: u8) -> Result<HciResponse> {
        self.wait_status(STAT_RESP_READY)?;
        self.consume_response(tid)
    }

    fn consume_response(&mut self, tid: u8) -> Result<HciResponse> {
        let raw = self.regs.pio_response_queue_port();
        let response = HciResponse::parse(raw);
        if let Some(trace) = self.trace {
            trace(format_args!(
                "Response raw=0x{:08x} TID={} expected_TID={} status=0x{:02x} data_len={}",
                raw, response.tid, tid, response.status, response.data_len
            ));
        }
        if response.tid != tid {
            return Err(I3cHciError::InvalidArgs);
        }
        let status = self.event_status();
        // DAA's terminal NACK may assert TRANSFER_ERR with a valid response.
        // Consume that summary only after preserving the detailed response.
        if !response.success() {
            self.ack_errors(STAT_TRANSFER_ERR);
        }
        self.check_status_errors(if response.success() {
            status
        } else {
            status & !STAT_TRANSFER_ERR
        })?;
        Ok(response)
    }

    /// Return live readiness plus latched errors. A FIFO-ready notification
    /// cannot authorize a read/write after its level has deasserted.
    fn event_status(&mut self) -> u32 {
        let (live, latched) = match self.irq_events {
            Some(events) => events.snapshot_io_status(|| self.regs.pio_intr_status()),
            None => (self.regs.pio_intr_status(), 0),
        };
        self.pending_irq_status |= (live | latched) & STAT_ALL_ERRORS;
        live | self.pending_irq_status
    }

    /// Inspect readiness once. Both latched and live fatal errors take precedence.
    fn try_status(&mut self, mask: u32) -> Result<bool> {
        let status = self.event_status();
        if mask == STAT_RESP_READY
            && status & STAT_RESP_READY != 0
            && status & (STAT_ALL_ERRORS & !STAT_TRANSFER_ERR) == 0
        {
            return Ok(true);
        }
        self.check_status_errors(status)?;
        Ok(status & mask != 0)
    }

    fn ack_errors(&mut self, mask: u32) {
        if let Some(events) = self.irq_events {
            events.ack_io_status(mask, || self.regs.write_pio_intr_status(mask));
        } else {
            self.regs.write_pio_intr_status(mask);
        }
        self.pending_irq_status &= !mask;
    }

    /// Preserve existing subscriptions when the ISR temporarily masks sources.
    pub(crate) fn enable_irq_signals(&self, mask: u32) {
        if let Some(events) = self.irq_events {
            events.rearm_io_signals(|saved| {
                self.regs.write_pio_intr_signal_enable(
                    self.regs.pio_intr_signal_enable() | saved | mask,
                );
            });
        } else {
            self.regs
                .write_pio_intr_signal_enable(self.regs.pio_intr_signal_enable() | mask);
        }
    }

    pub(crate) fn rearm_pio_irqs(&self, mask: u32) {
        if self.irq_events.is_some() {
            self.enable_irq_signals(mask);
        }
    }

    fn check_errors(&mut self) -> Result<()> {
        let status = self.event_status();
        self.check_status_errors(status)
    }

    fn check_status_errors(&mut self, status: u32) -> Result<()> {
        let errors = status & STAT_ALL_ERRORS;
        if errors != 0 {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "HCI{} PIO error before W1C: status=0x{:08x} errors=0x{:08x}",
                    self.regs.bus(),
                    status,
                    errors
                ));
            }
            self.ack_errors(errors);
            return Err(I3cHciError::TransferStatus(errors));
        }
        Ok(())
    }
}

#[must_use]
pub fn word_from_bytes(bytes: &[u8]) -> u32 {
    let mut word = 0;
    let mut i = 0;
    while i < bytes.len() {
        word |= (bytes[i] as u32) << (i as u32 * 8);
        i += 1;
    }
    word
}

pub fn bytes_from_word(word: u32, out: &mut [u8]) {
    let mut i = 0;
    while i < out.len() {
        out[i] = ((word >> (i as u32 * 8)) & 0xff) as u8;
        i += 1;
    }
}

/// Drain the advertised byte count, including a padded final FIFO word.
/// Truncation never leaves unread words for the next transaction.
fn drain_fifo(
    output: &mut [u8],
    count: usize,
    mut read_word: impl FnMut() -> Result<u32>,
) -> Result<usize> {
    let copy_len = output.len().min(count);
    let mut offset = 0;
    while offset < count {
        let word = read_word()?;
        let chunk = (count - offset).min(4);
        if offset < copy_len {
            let copied = chunk.min(copy_len - offset);
            bytes_from_word(word, &mut output[offset..offset + copied]);
        }
        offset += chunk;
    }
    if count > output.len() {
        return Err(I3cHciError::NoSpace);
    }
    Ok(copy_len)
}

impl<Y: FnMut(u32)> I3cHciPio<'_, Y> {
    fn wait_status(&mut self, mask: u32) -> Result<()> {
        let result = super::completion::poll(
            self.poll_budget.saturating_add(1),
            || {
                if self.try_status(mask)? {
                    return Ok(Some(()));
                }
                (self.yield_us)(1);
                Ok(None)
            },
            I3cHciError::Timeout,
        );
        if result == Err(I3cHciError::Timeout) {
            if let Some(trace) = self.trace {
                trace(format_args!(
                    "HCI{} timeout mask={:08x} status={:08x} pending={:08x}",
                    self.regs.bus(),
                    mask,
                    self.regs.pio_intr_status(),
                    self.pending_irq_status
                ));
                self.regs.trace_transfer_state("PIO timeout", trace);
            }
        }
        result
    }
    fn wait_register_clear(
        &mut self,
        read: fn(&I3cHciRegisters) -> u32,
        mask: u32,
        budget: u32,
    ) -> Result<()> {
        super::completion::poll(
            budget.saturating_add(1),
            || {
                if read(self.regs) & mask == 0 {
                    return Ok(Some(()));
                }
                (self.yield_us)(1);
                Ok(None)
            },
            I3cHciError::Timeout,
        )
    }
    pub(crate) fn poll_budget(&self) -> u32 {
        self.poll_budget
    }
}

impl<Y: FnMut(u32)> I3cHciPio<'_, Y> {
    pub(crate) fn trace_callback(&self) -> Option<fn(core::fmt::Arguments<'_>)> {
        self.trace
    }

    pub(crate) fn uses_irq_events(&self) -> bool {
        self.irq_events.is_some()
    }

    pub(crate) fn into_wait_context(self) -> (Y, u32) {
        (self.yield_us, self.poll_budget)
    }
}

enum PioPhase {
    Tx,
    Command,
    Response,
}
struct PioCommand<'a> {
    transfer: HciTransfer<'a>,
    tx_offset: usize,
    phase: PioPhase,
}

pub(crate) struct PioCommandOperation<'operation, 'regs, 'buffer, Y: FnMut(u32)> {
    pio: &'operation mut I3cHciPio<'regs, Y>,
    pending: PioCommand<'buffer>,
    completed: bool,
}
impl<Y: FnMut(u32)> super::completion::Operation for PioCommandOperation<'_, '_, '_, Y> {
    type Output = HciTransferResult;
    type Error = I3cHciError;
    fn advance(&mut self) -> Result<super::completion::Progress<HciTransferResult>> {
        let progress = self.pio.progress_command(&mut self.pending)?;
        if matches!(&progress, super::completion::Progress::Complete(_)) {
            self.completed = true;
        }
        Ok(progress)
    }
    fn on_poll_pending(&mut self) {
        (self.pio.yield_us)(1);
    }
}
impl<Y: FnMut(u32)> Drop for PioCommandOperation<'_, '_, '_, Y> {
    fn drop(&mut self) {
        if !self.completed {
            self.pio.command_faulted = true;
        }
    }
}
impl<Y: FnMut(u32)> I3cHciPio<'_, Y> {
    /// Synchronous multi-step paths need the same failure latch as operation
    /// guards: a timeout may leave a partial TX or an unconsumed response.
    fn command_sequence<T>(&mut self, sequence: impl FnOnce(&mut Self) -> Result<T>) -> Result<T> {
        self.require_command_ready()?;
        self.command_faulted = true;
        let value = sequence(self)?;
        self.command_faulted = false;
        Ok(value)
    }

    fn require_command_ready(&self) -> Result<()> {
        if self.command_faulted {
            Err(I3cHciError::RecoveryRequired)
        } else {
            Ok(())
        }
    }
}
