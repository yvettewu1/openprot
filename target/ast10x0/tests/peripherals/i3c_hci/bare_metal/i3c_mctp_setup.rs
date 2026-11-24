// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Test-side MCTP IBI PEC helpers; data packets are echoed without modification.

/// CRC-8 polynomial 0x07, including the I3C address/direction byte.
fn crc8(address_byte: u8, data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for byte in core::iter::once(address_byte).chain(data.iter().copied()) {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub fn ibi_payload(address: u8, mdb: u8) -> [u8; 2] {
    [mdb, crc8((address << 1) | 1, &[mdb])]
}
