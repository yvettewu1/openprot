// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI DCT v1 helpers.
//!
//! Zephyr's DCT v1 code reads four 32-bit words and decodes PID, DCR, and BCR.

use super::core::TableSection;
use super::error::{I3cHciError, Result};
use super::registers::I3cHciRegisters;

pub const DCT_V1_ENTRY_WORDS: u8 = 4;
pub const DCT_V1_ENTRY_SIZE: u8 = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DctEntryV1 {
    pub words: [u32; DCT_V1_ENTRY_WORDS as usize],
    pub pid: u64,
    pub dcr: u8,
    pub bcr: u8,
}

impl DctEntryV1 {
    #[must_use]
    pub const fn from_words(words: [u32; DCT_V1_ENTRY_WORDS as usize]) -> Self {
        Self {
            words,
            pid: ((words[0] as u64) << 16) | ((words[1] & 0xffff) as u64),
            dcr: (words[2] & 0xff) as u8,
            bcr: ((words[2] >> 8) & 0xff) as u8,
        }
    }
}

pub fn read_dct_entry_v1(
    regs: &I3cHciRegisters,
    section: TableSection,
    index: u8,
) -> Result<DctEntryV1> {
    if section.offset == 0
        || section.entry_size != DCT_V1_ENTRY_SIZE
        || u16::from(index) >= section.entries
    {
        return Err(I3cHciError::InvalidArgs);
    }

    let mut words = [0u32; DCT_V1_ENTRY_WORDS as usize];
    let mut word = 0;
    while word < DCT_V1_ENTRY_WORDS {
        words[word as usize] = regs
            .table_word(section.offset, section.entry_size, index, word)
            .ok_or(I3cHciError::InvalidArgs)?;
        word += 1;
    }

    Ok(DctEntryV1::from_words(words))
}
