// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Host regression tests exercising the production PIO code with a G7 FIFO model.
//! Run: rustc --edition 2024 --test host_ibi.rs -o /tmp/hci-ibi-tests && /tmp/hci-ibi-tests
#![allow(dead_code)]

#[path = "../../../../peripherals/i3c_hci/cmd.rs"]
mod cmd;
#[path = "../../../../peripherals/i3c_hci/constants.rs"]
mod constants;
#[path = "../../../../peripherals/i3c_hci/error.rs"]
mod error;
#[path = "../../../../peripherals/i3c_hci/pio.rs"]
mod pio;

// Minimal master interface for this standalone IBI FIFO model. The production
// transfer definitions and validation live in master.rs; validation coverage
// uses those definitions in pio/target.rs. Fail if this fixture is used to submit.
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
        panic!("this IBI-only fixture does not support command submission")
    }
}

mod irq {
    pub struct IsrEvents;
    impl IsrEvents {
        pub fn take_io_status(&self) -> u32 {
            0
        }
    }
    pub fn isr_events(_: usize) -> Option<&'static IsrEvents> {
        None
    }
}

mod registers {
    use std::{cell::RefCell, collections::VecDeque};
    pub struct I3cHciRegisters {
        words: RefCell<VecDeque<(bool, u32)>>,
    }
    impl I3cHciRegisters {
        pub fn new(words: Vec<(bool, u32)>) -> Self {
            Self {
                words: RefCell::new(words.into()),
            }
        }
        pub fn empty(&self) -> bool {
            self.words.borrow().is_empty()
        }
        pub fn pio_intr_status(&self) -> u32 {
            // G7: payload availability must NOT assert STATUS_THLD.
            if self.words.borrow().front().is_some_and(|entry| entry.0) {
                super::pio::STAT_IBI_STATUS_THLD
            } else {
                0
            }
        }
        pub fn pio_ibi_port(&self) -> u32 {
            self.words
                .borrow_mut()
                .pop_front()
                .expect("IBI FIFO underflow")
                .1
        }
        pub fn bus(&self) -> u8 {
            0
        }
        pub fn aspeed_slave_cap_ctrl(&self) -> u32 {
            0
        }
        pub fn pio_response_queue_port(&self) -> u32 {
            panic!("unexpected response read")
        }
        pub fn pio_rx_data_port(&self) -> u32 {
            panic!("unexpected RX read")
        }
        pub fn hc_control(&self) -> u32 {
            0
        }
        pub fn intr_status(&self) -> u32 {
            0
        }
        pub fn pio_intr_signal_enable(&self) -> u32 {
            0
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
            panic!("unexpected command")
        }
        pub fn write_pio_tx_data_port(&self, _: u32) {
            panic!("unexpected TX")
        }
        pub fn write_pio_intr_signal_enable(&self, _: u32) {}
        pub fn write_pio_intr_status(&self, _: u32) {}
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
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_timeout_us(3);
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
    let mut driver = pio::I3cHciPio::new(&regs, |_| {}).with_timeout_us(3);
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
