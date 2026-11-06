// Test EIDs are independent of the I3C dynamic addresses. Null destination
// addresses the directly attached endpoint for this receive-queue test.
const MCTP_DEST_EID: u8 = 0x00;
const MCTP_SOURCE_EID: u8 = 0x08;
// Vendor-defined PCI message, IC=0; ASPEED vendor ID, most significant byte first.
// These identify the test payload, not an implemented ASPEED command protocol.
const MCTP_MESSAGE_TYPE: u8 = 0x7e;
const MCTP_VENDOR_ID: u16 = 0x1a03;
const MCTP_HEADER_LEN: usize = 7; // Transport (4), message type (1), vendor ID (2).
const MCTP_WRITE_LEN: usize = MCTP_HEADER_LEN + PRIVATE_WRITE_DATA.len() + 1;

const PRIVATE_WRITE_DATA: [u8; 8] = [0xaa, 0xee, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

pub fn prepare_mctp_packet(dynamic_addr: u8) -> [u8; MCTP_WRITE_LEN] {
    let mut packet = [0u8; MCTP_WRITE_LEN];
    packet[..MCTP_HEADER_LEN].copy_from_slice(&[
        0x01, // Transport header version 1.
        MCTP_DEST_EID,
        MCTP_SOURCE_EID,
        0xc8, // SOM=1, EOM=1, sequence=0, tag owner=1, tag=0.
        MCTP_MESSAGE_TYPE,
        (MCTP_VENDOR_ID >> 8) as u8,
        MCTP_VENDOR_ID as u8,
    ]);
    let pec_index = packet.len() - 1;
    packet[MCTP_HEADER_LEN..pec_index].copy_from_slice(&PRIVATE_WRITE_DATA);
    // DSP0233 PEC: CRC-8 polynomial 0x07, initial value 0, over the
    // destination address with W=0 followed by the MCTP header and message.
    // The address byte participates in PEC but is not part of the payload.
    let mut pec = 0u8;
    for byte in core::iter::once(dynamic_addr << 1).chain(packet[..pec_index].iter().copied()) {
        pec ^= byte;
        for _ in 0..8 {
            pec = if pec & 0x80 != 0 {
                (pec << 1) ^ 0x07
            } else {
                pec << 1
            };
        }
    }
    packet[pec_index] = pec;
    packet
}