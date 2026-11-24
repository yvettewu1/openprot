// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Host regression tests exercising production PIO transfers and IBI with a G7 FIFO model.
//! Run: rustc --edition 2024 --test host_ibi.rs -o /tmp/hci-ibi-tests && /tmp/hci-ibi-tests
#![allow(dead_code)]

#[path = "../../../../../peripherals/i3c_hci/cmd.rs"]
mod cmd;
#[path = "../../../../../peripherals/i3c_hci/completion.rs"]
mod completion;
#[path = "../../../../../peripherals/i3c_hci/constants.rs"]
mod constants;
#[path = "../../../../../peripherals/i3c_hci/error.rs"]
mod error;
#[path = "../../../../../peripherals/i3c_hci/pio.rs"]
mod pio;

// Minimal master types for this standalone FIFO model. All transfer fixtures
// below are valid encoder outputs. Production validation lives in master.rs
// and is covered by driver_logic; this model tests FIFO/progress/error behavior.
mod master {
    pub struct HciTransfer<'a> {
        pub command: super::cmd::HciCommand,
        pub tx: &'a [u8],
        pub rx: &'a mut [u8],
    }
    pub struct HciTransferResult {
        pub response: super::cmd::HciResponse,
        pub rx_len: usize,
    }
    pub fn validate_transfer(_: &HciTransfer<'_>) -> super::error::Result<()> {
        Ok(())
    }
}

mod irq {
    use std::cell::Cell;
    std::thread_local! {
        static LATCH: Cell<u32> = const { Cell::new(0) };
        static MASKED: Cell<u32> = const { Cell::new(0) };
    }
    pub fn reset() {
        LATCH.set(0);
        MASKED.set(0);
    }
    pub fn inject(status: u32, masked: u32) {
        LATCH.set(LATCH.get() | status);
        MASKED.set(MASKED.get() | masked);
    }
    pub struct IsrEvents;
    impl IsrEvents {
        pub fn take_io_status(&self) -> u32 {
            LATCH.replace(0)
        }
        pub fn snapshot_io_status(&self, read: impl FnOnce() -> u32) -> (u32, u32) {
            (read(), self.take_io_status())
        }
        pub fn ack_io_status(&self, mask: u32, acknowledge: impl FnOnce()) {
            acknowledge();
            LATCH.set(LATCH.get() & !mask);
        }
        pub fn rearm_io_signals(&self, rearm: impl FnOnce(u32)) {
            rearm(MASKED.replace(0));
        }
    }
    static EVENTS: IsrEvents = IsrEvents;
    pub fn isr_events(bus: usize) -> Option<&'static IsrEvents> {
        (bus == 0).then_some(&EVENTS)
    }
}

mod registers {
    use std::{
        cell::{Cell, RefCell},
        collections::VecDeque,
    };
    pub struct I3cHciRegisters {
        words: RefCell<VecDeque<(bool, u32)>>,
        signal_enable: Cell<u32>,
        pub status: Cell<u32>,
        pub ibi_reads_until_error: Cell<Option<usize>>,
        pub commands_written: Cell<usize>,
        pub tx_words_written: Cell<usize>,
        pub tx_words: RefCell<Vec<u32>>,
        pub responses: RefCell<VecDeque<u32>>,
        pub rx_words: RefCell<VecDeque<u32>>,
    }
    impl I3cHciRegisters {
        pub fn new(words: Vec<(bool, u32)>) -> Self {
            super::irq::reset();
            Self {
                words: RefCell::new(words.into()),
                signal_enable: Cell::new(0),
                status: Cell::new(0),
                ibi_reads_until_error: Cell::new(None),
                commands_written: Cell::new(0),
                tx_words_written: Cell::new(0),
                tx_words: RefCell::new(Vec::new()),
                responses: RefCell::new(VecDeque::new()),
                rx_words: RefCell::new(VecDeque::new()),
            }
        }
        pub fn append(&self, words: Vec<(bool, u32)>) {
            self.words.borrow_mut().extend(words);
        }
        pub fn empty(&self) -> bool {
            self.words.borrow().is_empty()
        }
        pub fn pio_intr_status(&self) -> u32 {
            // G7: payload availability must NOT assert STATUS_THLD.
            let mut status = self.status.get();
            if self.ibi_reads_until_error.get() == Some(0) {
                status |= super::pio::STAT_TRANSFER_ERR;
            }
            if self.words.borrow().front().is_some_and(|entry| entry.0) {
                status |= super::pio::STAT_IBI_STATUS_THLD;
            }
            if !self.responses.borrow().is_empty() {
                status |= super::pio::STAT_RESP_READY;
            }
            status
        }
        pub fn pio_ibi_port(&self) -> u32 {
            if let Some(remaining) = self.ibi_reads_until_error.get() {
                self.ibi_reads_until_error
                    .set(Some(remaining.saturating_sub(1)));
            }
            self.words
                .borrow_mut()
                .pop_front()
                .expect("IBI FIFO underflow")
                .1
        }
        pub fn trace_transfer_state(&self, _: &str, _: fn(core::fmt::Arguments<'_>)) {}
        pub fn bus(&self) -> u8 {
            0
        }
        pub fn aspeed_slave_cap_ctrl(&self) -> u32 {
            0
        }
        pub fn pio_response_queue_port(&self) -> u32 {
            self.responses
                .borrow_mut()
                .pop_front()
                .expect("unexpected response read")
        }
        pub fn pio_rx_data_port(&self) -> u32 {
            self.rx_words
                .borrow_mut()
                .pop_front()
                .expect("unexpected RX read")
        }
        pub fn hc_control(&self) -> u32 {
            0
        }
        pub fn intr_status(&self) -> u32 {
            0
        }
        pub fn pio_intr_signal_enable(&self) -> u32 {
            self.signal_enable.get()
        }
        pub fn pio_intr_status_enable(&self) -> u32 {
            0
        }
        pub fn pio_queue_size(&self) -> u32 {
            0
        }
        pub fn pio_queue_thld_ctrl(&self) -> u32 {
            0
        }
        pub fn present_state(&self) -> u32 {
            0
        }
        pub fn reset_control(&self) -> u32 {
            0
        }
        pub fn set_hc_control(&self, _: u32) {}
        pub fn write_pio_command_queue_port(&self, _: u32) {
            self.commands_written.set(self.commands_written.get() + 1);
        }
        pub fn write_pio_tx_data_port(&self, word: u32) {
            self.tx_words_written.set(self.tx_words_written.get() + 1);
            self.tx_words.borrow_mut().push(word);
        }
        pub fn write_pio_intr_signal_enable(&self, mask: u32) {
            self.signal_enable.set(mask);
        }
        pub fn write_pio_intr_status(&self, mask: u32) {
            self.status.set(self.status.get() & !mask);
            if mask & super::pio::STAT_TRANSFER_ERR != 0 {
                self.ibi_reads_until_error.set(None);
            }
        }
        pub fn write_reset_control(&self, _: u32) {}
    }
}

fn segment(words: &mut Vec<(bool, u32)>, addr: u8, last: bool, error: bool, data: &[u8]) {
    words.push((
        true,
        ((addr as u32) << 9)
            | (1 << 8)
            | ((last as u32) << 24)
            | ((error as u32) << 30)
            | data.len() as u32,
    ));
    for chunk in data.chunks(4) {
        words.push((false, pio::word_from_bytes(chunk)));
    }
}

#[test]
fn short_notification_reads_payload_after_status_bit_clears() {
    let mut words = vec![];
    segment(&mut words, 9, true, false, &[0xae, 0x12]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_poll_budget(3);
    let mut payload = [0; 8];
    let (status, len) = driver.next_ibi(&mut payload).unwrap();
    assert!(status.last);
    assert_eq!(len, 2);
    assert_eq!(&payload[..len], &[0xae, 0x12]);
    assert!(regs.empty());
}

#[test]
fn assembles_segments_without_copying_word_padding() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[1, 2, 3]);
    segment(&mut words, 9, true, false, &[4, 5, 6, 7, 8]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    let mut payload = [0; 8];
    let (status, len) = driver.next_ibi(&mut payload).unwrap();
    assert_eq!(status.data_len, 5); // Final descriptor remains unmodified.
    assert_eq!(len, 8);
    assert_eq!(payload, [1, 2, 3, 4, 5, 6, 7, 8]);
    assert!(regs.empty());
}

#[test]
fn overflow_drains_all_segments_and_preserves_next_event() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[1, 2, 3, 4, 5]);
    segment(&mut words, 9, true, false, &[6, 7]);
    segment(&mut words, 9, true, false, &[0xae]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    let mut payload = [0; 2];
    assert_eq!(
        driver.next_ibi(&mut payload),
        Err(error::I3cHciError::NoSpace)
    );
    assert_eq!(driver.next_ibi(&mut payload).unwrap().1, 1);
    assert_eq!(payload[0], 0xae);
    assert!(regs.empty());
}

#[test]
fn nonfinal_error_is_not_hidden_by_successful_final_status() {
    let mut words = vec![];
    segment(&mut words, 9, false, true, &[1]);
    segment(&mut words, 9, true, false, &[2]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    assert!(matches!(
        driver.next_ibi(&mut [0; 8]),
        Err(error::I3cHciError::TransferStatus(_))
    ));
    assert!(regs.empty());
}

#[test]
fn changed_source_is_rejected() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[1]);
    segment(&mut words, 10, true, false, &[2]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    assert!(matches!(
        driver.next_ibi(&mut [0; 8]),
        Err(error::I3cHciError::TransferStatus(_))
    ));
}

#[test]
fn incomplete_transaction_never_returns_success() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[0xae]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_poll_budget(3);
    assert_eq!(
        driver.next_ibi(&mut [0; 8]),
        Err(error::I3cHciError::Timeout)
    );
}

#[test]
fn hotjoin_without_last_bit_does_not_wait_for_another_descriptor() {
    let regs = registers::I3cHciRegisters::new(vec![(true, 2 << 9)]);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    let (status, len) = driver.next_ibi(&mut []).unwrap();
    assert!(!status.read);
    assert_eq!(status.target_addr, 2);
    assert_eq!(len, 0);
}

#[test]
fn reported_hardware_error_41001304_is_rejected_after_draining() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[0xff; 64]);
    segment(&mut words, 9, true, true, &[0xff; 4]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    assert_eq!(
        driver.next_ibi(&mut [0; 128]),
        Err(error::I3cHciError::TransferStatus(0x41001304))
    );
    assert!(regs.empty());
}

#[test]
fn poll_ibi_returns_between_segments_without_yielding() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[0xae]);
    segment(&mut words, 9, true, false, &[0x2b]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| panic!("poll must not wait"));
    let mut payload = [0; 2];
    assert!(driver.poll_ibi(&mut payload).unwrap().is_none());
    assert_eq!(payload[0], 0xae);
    assert_eq!(driver.poll_ibi(&mut payload).unwrap().unwrap().1, 2);
    assert_eq!(payload, [0xae, 0x2b]);
    assert!(regs.empty());
}

#[test]
fn segmented_wait_budget_resets_after_progress() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[0xae]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut yields = 0;
    let mut driver = pio::I3cHciPio::new(&regs, |_| {
        yields += 1;
        let mut tail = vec![];
        segment(&mut tail, 9, true, false, &[0x2b]);
        regs.append(tail);
    })
    .with_poll_budget(2);
    let mut payload = [0; 2];
    assert_eq!(driver.next_ibi(&mut payload).unwrap().1, 2);
    drop(driver);
    assert_eq!(yields, 1);
    assert_eq!(payload, [0xae, 0x2b]);
    assert!(regs.empty());
}

#[test]
fn partial_poll_rejects_buffer_changes_without_consuming_tail() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[0xae]);
    segment(&mut words, 9, true, false, &[0x2b]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| panic!("poll must not wait"));
    let mut payload = [0; 2];
    let mut other = [0; 2];
    assert!(driver.poll_ibi(&mut payload).unwrap().is_none());
    assert_eq!(
        driver.poll_ibi(&mut other),
        Err(error::I3cHciError::InvalidArgs)
    );
    assert!(!regs.empty());
    assert_eq!(driver.poll_ibi(&mut payload).unwrap().unwrap().1, 2);
    assert_eq!(payload, [0xae, 0x2b]);
}

#[test]
fn polling_drains_overflow_then_accepts_next_notification() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[1, 2, 3]);
    segment(&mut words, 9, true, false, &[4]);
    segment(&mut words, 9, true, false, &[0xae, 0x2b]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| panic!("poll must not wait"));
    let mut payload = [0; 2];
    assert!(driver.poll_ibi(&mut payload).unwrap().is_none());
    assert_eq!(
        driver.poll_ibi(&mut payload),
        Err(error::I3cHciError::NoSpace)
    );
    assert_eq!(driver.poll_ibi(&mut payload).unwrap().unwrap().1, 2);
    assert_eq!(payload, [0xae, 0x2b]);
    assert!(regs.empty());
}

#[test]
fn deferred_ibi_rearms_each_segment_only_when_irq_delivery_selected() {
    for use_irq in [false, true] {
        let mut words = vec![];
        segment(&mut words, 9, false, false, &[0xae]);
        segment(&mut words, 9, true, false, &[0x2b]);
        let regs = registers::I3cHciRegisters::new(words);
        let driver = pio::I3cHciPio::new(&regs, |_| panic!("poll must not wait"));
        let mut driver = if use_irq {
            driver.with_irq_events(0)
        } else {
            driver
        };
        let mut payload = [0; 2];
        for complete in [false, true] {
            regs.write_pio_intr_signal_enable(0); // Model ISR masking the source.
            assert_eq!(driver.poll_ibi(&mut payload).unwrap().is_some(), complete);
            let expected = if use_irq {
                pio::STAT_IBI_STATUS_THLD | pio::STAT_ALL_ERRORS
            } else {
                0
            };
            assert_eq!(regs.pio_intr_signal_enable(), expected);
        }
        assert_eq!(payload, [0xae, 0x2b]);
    }
}

#[test]
fn discard_partial_ibi_waits_for_tail_and_preserves_next_event() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[1, 2, 3]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    let mut original = [0; 8];
    assert!(driver.poll_ibi(&mut original).unwrap().is_none());
    assert!(!driver.discard_partial_ibi().unwrap());
    let mut tail = vec![];
    segment(&mut tail, 9, true, false, &[4, 5]);
    segment(&mut tail, 9, true, false, &[0xae, 0x2b]);
    regs.append(tail);
    assert!(driver.discard_partial_ibi().unwrap());
    let mut replacement = [0; 2];
    assert_eq!(driver.poll_ibi(&mut replacement).unwrap().unwrap().1, 2);
    assert_eq!(replacement, [0xae, 0x2b]);
    assert_eq!(&original[..3], &[1, 2, 3]);
    assert!(regs.empty());
}

#[test]
fn discard_without_partial_event_does_not_pop_queued_notification() {
    let mut words = vec![];
    segment(&mut words, 9, true, false, &[0xae]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    assert!(driver.discard_partial_ibi().unwrap());
    let mut payload = [0; 1];
    assert_eq!(driver.poll_ibi(&mut payload).unwrap().unwrap().1, 1);
    assert_eq!(payload, [0xae]);
}

#[test]
fn oversized_controller_response_drains_full_rx_before_error() {
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_CMD_QUEUE_READY);
    regs.responses.borrow_mut().push_back(0x01000008);
    regs.rx_words.borrow_mut().extend([0x04030201, 0x08070605]);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_poll_budget(1);
    let command = cmd::HciCommand::V1(
        cmd::HciCommandV1::i3c_private_read(9, 1, 4, 0)
            .unwrap()
            .with_roc(),
    );
    let mut output = [0; 4];
    assert!(matches!(
        driver.submit_raw(master::HciTransfer {
            command,
            tx: &[],
            rx: &mut output,
        }),
        Err(error::I3cHciError::NoSpace)
    ));
    assert_eq!(output, [1, 2, 3, 4]);
    assert!(regs.rx_words.borrow().is_empty());
}

#[test]
fn daa_assignment_count_does_not_read_rx_fifo() {
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_CMD_QUEUE_READY);
    regs.responses.borrow_mut().push_back(0x50000001);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_poll_budget(1);
    let command = cmd::HciCommand::V1(cmd::HciCommandV1 {
        words: [0xc4090382, 0],
        tid: 0,
        kind: cmd::HciCommandKind::AddressAssignment,
        read: true,
        data_len: 0,
    });
    let response = driver
        .submit_raw(master::HciTransfer {
            command,
            tx: &[],
            rx: &mut [],
        })
        .unwrap();
    assert_eq!(response.response.data_len, 1);
    assert_eq!(response.rx_len, 0);
    // RX reads panic: the assignment count is not one byte of FIFO data.
}

#[test]
fn failed_no_response_submission_blocks_reuse_of_partial_tx() {
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_TX_THLD); // TX accepted, command queue never ready.
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_poll_budget(1);
    let command =
        cmd::HciCommand::V1(cmd::HciCommandV1::i3c_private_write_regular(9, 1, 1, 0).unwrap());
    assert_eq!(
        driver.submit_no_response(master::HciTransfer {
            command,
            tx: &[0xae],
            rx: &mut [],
        }),
        Err(error::I3cHciError::Timeout)
    );
    assert_eq!(regs.tx_words_written.get(), 1);
    assert_eq!(
        driver.submit_no_response(master::HciTransfer {
            command,
            tx: &[0xae],
            rx: &mut [],
        }),
        Err(error::I3cHciError::RecoveryRequired)
    );
    assert_eq!(regs.tx_words_written.get(), 1);
}

#[test]
fn failed_paired_submission_blocks_another_pair() {
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_CMD_QUEUE_READY); // Both queued, no response.
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_poll_budget(1);
    let first = cmd::HciCommand::V2(cmd::HciCommandV2::daa_read_pid(1, 8).unwrap().with_roc());
    let second = cmd::HciCommand::V2(
        cmd::HciCommandV2::daa_assign_address(2, 9)
            .unwrap()
            .with_roc()
            .with_toc(),
    );
    assert!(matches!(
        driver.submit_pair_raw(first, second, &mut [0; 8]),
        Err(error::I3cHciError::Timeout)
    ));
    let count = regs.commands_written.get();
    assert_eq!(count, 8);
    assert!(matches!(
        driver.submit_pair_raw(first, second, &mut [0; 8]),
        Err(error::I3cHciError::RecoveryRequired)
    ));
    assert_eq!(regs.commands_written.get(), count);
}

#[test]
fn interrupted_ibi_payload_cannot_be_reinterpreted_as_status() {
    let mut words = vec![];
    segment(&mut words, 9, true, false, &[0xae, 1, 2, 3, 4, 5, 6, 7]);
    let regs = registers::I3cHciRegisters::new(words);
    regs.ibi_reads_until_error.set(Some(2)); // Status and first payload word.
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    let mut payload = [0; 8];
    assert!(matches!(
        driver.poll_ibi(&mut payload),
        Err(error::I3cHciError::TransferStatus(_))
    ));
    assert!(matches!(
        driver.poll_ibi(&mut payload),
        Err(error::I3cHciError::RecoveryRequired)
    ));
    assert_eq!(
        driver.discard_partial_ibi(),
        Err(error::I3cHciError::RecoveryRequired)
    );
    assert!(!regs.empty()); // Uncertain tail remains untouched until reset.
}

#[test]
fn raw_segment_api_cannot_bypass_partial_or_faulted_ibi_ownership() {
    let mut words = vec![];
    segment(&mut words, 9, false, false, &[0xae]);
    segment(&mut words, 9, true, false, &[0x2b]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {});
    let mut payload = [0; 8];
    assert!(driver.poll_ibi(&mut payload).unwrap().is_none());
    assert_eq!(
        driver.next_ibi_status(&mut payload),
        Err(error::I3cHciError::Busy)
    );
    assert_eq!(driver.poll_ibi(&mut payload).unwrap().unwrap().1, 2);
    assert_eq!(&payload[..2], &[0xae, 0x2b]);

    let mut words = vec![];
    segment(&mut words, 9, true, false, &[1, 2, 3, 4, 5]);
    regs.append(words);
    regs.ibi_reads_until_error.set(Some(2));
    assert!(matches!(
        driver.next_ibi_status(&mut payload),
        Err(error::I3cHciError::TransferStatus(_))
    ));
    assert!(matches!(
        driver.poll_ibi(&mut payload),
        Err(error::I3cHciError::RecoveryRequired)
    ));
}

#[test]
fn stale_latched_response_ready_never_pops_empty_response_fifo() {
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_CMD_QUEUE_READY);
    irq::inject(pio::STAT_RESP_READY, pio::STAT_RESP_READY);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {})
        .with_irq_events(0)
        .with_poll_budget(1);
    let command = cmd::HciCommand::V1(
        cmd::HciCommandV1::i3c_private_read(9, 1, 0, 0)
            .unwrap()
            .with_roc(),
    );
    assert!(matches!(
        driver.submit_raw(master::HciTransfer {
            command,
            tx: &[],
            rx: &mut [],
        }),
        Err(error::I3cHciError::Timeout)
    ));
}

#[test]
fn live_ibi_ready_does_not_hide_latched_fifo_error() {
    let mut words = vec![];
    segment(&mut words, 9, true, false, &[0xae, 0x2b]);
    let regs = registers::I3cHciRegisters::new(words);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_irq_events(0);
    irq::inject(pio::STAT_PERR_RX_UFLOW, pio::STAT_IBI_STATUS_THLD);
    assert!(matches!(driver.poll_ibi(&mut [0; 8]),
        Err(error::I3cHciError::TransferStatus(bits)) if bits == pio::STAT_PERR_RX_UFLOW));
    assert!(!regs.empty());
}

#[test]
fn irq_transfer_error_does_not_hide_terminal_daa_response() {
    use completion::Operation;
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_CMD_QUEUE_READY);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_irq_events(0);
    let command = cmd::HciCommand::V1(cmd::HciCommandV1 {
        words: [0xc4090382, 0],
        tid: 0,
        kind: cmd::HciCommandKind::AddressAssignment,
        read: true,
        data_len: 0,
    });
    let mut rx = [];
    let mut operation = driver
        .begin_command(master::HciTransfer {
            command,
            tx: &[],
            rx: &mut rx,
        })
        .unwrap();
    assert!(matches!(
        operation.advance(),
        Ok(completion::Progress::Advanced)
    ));
    assert!(matches!(
        operation.advance(),
        Ok(completion::Progress::Advanced)
    ));
    regs.responses.borrow_mut().push_back(0x50000001);
    irq::inject(
        pio::STAT_RESP_READY | pio::STAT_TRANSFER_ERR,
        pio::STAT_RESP_READY,
    );
    match operation.advance().unwrap() {
        completion::Progress::Complete(result) => assert_eq!(result.response.raw, 0x50000001),
        _ => panic!("expected terminal DAA response"),
    }
}

#[test]
fn command_completion_restores_ibi_subscription_masked_by_isr() {
    let regs = registers::I3cHciRegisters::new(vec![]);
    regs.status.set(pio::STAT_CMD_QUEUE_READY);
    regs.responses.borrow_mut().push_back(0x01000000);
    let subscribed = pio::STAT_RESP_READY | pio::STAT_IBI_STATUS_THLD;
    irq::inject(pio::STAT_RESP_READY, subscribed);
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_irq_events(0);
    let command = cmd::HciCommand::V1(
        cmd::HciCommandV1::i3c_private_read(9, 1, 0, 0)
            .unwrap()
            .with_roc(),
    );
    driver
        .submit_raw(master::HciTransfer {
            command,
            tx: &[],
            rx: &mut [],
        })
        .unwrap();
    assert_eq!(regs.pio_intr_signal_enable() & subscribed, subscribed);
}

#[test]
fn raw_ibi_rearms_after_full_drain_even_with_small_output() {
    for capacity in [1, 2] {
        let mut words = vec![];
        segment(&mut words, 9, true, false, &[0xae, 0x2b]);
        let regs = registers::I3cHciRegisters::new(words);
        let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_irq_events(0);
        let subscribed = pio::STAT_RESP_READY | pio::STAT_IBI_STATUS_THLD;
        irq::inject(pio::STAT_IBI_STATUS_THLD, subscribed);
        let mut payload = [0; 2];
        let result = driver.next_ibi_status(&mut payload[..capacity]);
        if capacity == 1 {
            assert_eq!(result, Err(error::I3cHciError::NoSpace));
        } else {
            assert!(result.is_ok());
            assert_eq!(payload, [0xae, 0x2b]);
        }
        assert!(regs.empty());
        assert_eq!(regs.pio_intr_signal_enable() & subscribed, subscribed);
    }
}

// IRQ delivery still uses task-context polling in bare-metal tests. Exercise
// the same sequence with and without latch subscription, without a kernel wait.
#[test]
fn repeated_write_ibi_read_preserves_payloads_with_polling_or_irq_delivery() {
    for use_irq in [false, true] {
        let regs = registers::I3cHciRegisters::new(vec![]);
        regs.status
            .set(pio::STAT_CMD_QUEUE_READY | pio::STAT_TX_THLD);
        let driver = pio::I3cHciPio::new(&regs, |_| panic!("ready sequence must not wait"))
            .with_poll_budget(1);
        let mut driver = if use_irq {
            driver.with_irq_events(0)
        } else {
            driver
        };
        let subscriptions = pio::STAT_RESP_READY | pio::STAT_IBI_STATUS_THLD;
        let stale = pio::STAT_PERR_RESP_UFLOW | pio::STAT_RESP_READY;
        if !use_irq {
            // An unsubscribed engine must neither consume this stale latch
            // nor restore its signal mask. Only live hardware drives polling.
            irq::inject(stale, subscriptions);
        }

        for round in 0..6u8 {
            let write_tid = round * 2;
            let read_tid = write_tid + 1;
            let written = [0x11, round, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
            regs.responses
                .borrow_mut()
                .push_back(u32::from(write_tid) << 24);
            if use_irq {
                irq::inject(pio::STAT_RESP_READY, subscriptions);
                regs.write_pio_intr_signal_enable(0);
            }
            let command = cmd::HciCommand::V1(
                cmd::HciCommandV1::i3c_private_write_regular(9, write_tid, written.len(), 0)
                    .unwrap()
                    .with_roc()
                    .with_toc(),
            );
            let response = driver
                .submit(master::HciTransfer {
                    command,
                    tx: &written,
                    rx: &mut [],
                })
                .unwrap();
            assert_eq!(response.rx_len, 0);
            assert!(regs.responses.borrow().is_empty());
            assert_eq!(
                &regs.tx_words.borrow()[usize::from(round) * 2..],
                &[
                    pio::word_from_bytes(&written[..4]),
                    pio::word_from_bytes(&written[4..])
                ],
            );

            let mut words = vec![];
            segment(&mut words, 9, true, false, &[0xae, 0x2b]);
            regs.append(words);
            if use_irq {
                irq::inject(pio::STAT_IBI_STATUS_THLD, subscriptions);
                regs.write_pio_intr_signal_enable(0);
            }
            let mut notification = [0; 2];
            let (status, length) = driver.poll_ibi(&mut notification).unwrap().unwrap();
            assert_eq!(status.target_addr, 9);
            assert_eq!(length, 2);
            assert_eq!(notification, [0xae, 0x2b]);
            assert!(regs.empty());
            assert!(driver.poll_ibi(&mut notification).unwrap().is_none());

            let expected = [0xaa, round, 0xcc, 0xdd, 0xee, 0x12, 0x34, 0x56];
            regs.rx_words
                .borrow_mut()
                .extend(expected.chunks(4).map(pio::word_from_bytes));
            regs.responses
                .borrow_mut()
                .push_back((u32::from(read_tid) << 24) | 8);
            if use_irq {
                irq::inject(pio::STAT_RESP_READY, subscriptions);
                regs.write_pio_intr_signal_enable(0);
            }
            let command = cmd::HciCommand::V1(
                cmd::HciCommandV1::i3c_private_read(9, read_tid, expected.len(), 0)
                    .unwrap()
                    .with_roc()
                    .with_toc(),
            );
            let mut received = [0; 8];
            let response = driver
                .submit(master::HciTransfer {
                    command,
                    tx: &[],
                    rx: &mut received,
                })
                .unwrap();
            assert_eq!(response.rx_len, expected.len());
            assert_eq!(received, expected);
            assert!(regs.responses.borrow().is_empty());
            assert!(regs.rx_words.borrow().is_empty());
            assert_eq!(
                regs.pio_intr_signal_enable(),
                if use_irq {
                    subscriptions | pio::STAT_ALL_ERRORS
                } else {
                    0
                },
            );
        }
        assert_eq!(regs.tx_words_written.get(), 12);
        assert_eq!(regs.commands_written.get(), 24);
        if !use_irq {
            assert_eq!(irq::isr_events(0).unwrap().take_io_status(), stale);
        }
    }
}

#[test]
fn target_response_drains_short_rx_with_polling_or_irq_delivery() {
    const STAT_RX_THLD: u32 = 1 << 1;

    for use_irq in [false, true] {
        let regs = registers::I3cHciRegisters::new(vec![]);
        let driver = pio::I3cHciPio::new(&regs, |_| panic!("target poll must not wait"));
        let mut driver = if use_irq {
            driver.with_irq_events(0)
        } else {
            driver
        };
        // Completion authorizes the tail even when RX_THLD is never asserted.
        regs.responses.borrow_mut().push_back((1 << 27) | 2);
        regs.rx_words.borrow_mut().push_back(0x00002bae);
        if use_irq {
            irq::inject(pio::STAT_RESP_READY, pio::STAT_RESP_READY);
        }
        let response = driver.target_poll_response().unwrap().unwrap();
        assert!(response.target_received);
        assert_eq!(response.data_len, 2);
        assert_eq!(regs.pio_intr_status() & STAT_RX_THLD, 0);
        assert_eq!(
            driver.target_read_rx(&mut [0; 1], response.data_len),
            Err(error::I3cHciError::NoSpace)
        );
        assert!(regs.rx_words.borrow().is_empty());
        assert!(driver.target_poll_response().unwrap().is_none());

        // The oversized previous record must not pollute the following one.
        regs.responses.borrow_mut().push_back((1 << 27) | 2);
        regs.rx_words.borrow_mut().push_back(0x00002211);
        let response = driver.target_poll_response().unwrap().unwrap();
        let mut rx = [0; 2];
        assert_eq!(
            driver.target_read_rx(&mut rx, response.data_len).unwrap(),
            2
        );
        assert_eq!(rx, [0x11, 0x22]);
        assert!(regs.rx_words.borrow().is_empty());
    }
}

#[test]
fn target_response_ready_cannot_hide_live_or_latched_fifo_errors() {
    for use_irq in [false, true] {
        let regs = registers::I3cHciRegisters::new(vec![]);
        let driver = pio::I3cHciPio::new(&regs, |_| panic!("target poll must not wait"));
        let mut driver = if use_irq {
            driver.with_irq_events(0)
        } else {
            driver
        };
        regs.responses.borrow_mut().push_back((1 << 27) | 2);
        regs.rx_words.borrow_mut().push_back(0x00002bae);
        let error = pio::STAT_PERR_RX_UFLOW;
        if use_irq {
            irq::inject(error | pio::STAT_RESP_READY, pio::STAT_RESP_READY);
        } else {
            regs.status.set(error);
        }
        assert_eq!(
            driver.target_poll_response(),
            Err(error::I3cHciError::TransferStatus(error))
        );
        assert_eq!(regs.responses.borrow().len(), 1);
        assert_eq!(regs.rx_words.borrow().len(), 1);
    }
}
