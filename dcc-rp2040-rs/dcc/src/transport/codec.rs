use crate::transport::{Packet, PacketError, is_mf_extended_address};
use heapless::Vec;

/// Largest frame outside the advanced extended address partitions (S-9.2.1).
const MAX_EXTENDED_LEN: usize = 6;

pub(crate) struct DccCodec {}

impl DccCodec {
    /// Attempts to decode the packet.
    pub fn decode(raw: &[u8]) -> Result<Option<Packet>, PacketError> {
        // packets must be at least 3 bytes
        if raw.len() < 3 {
            return Err(PacketError::Undersize);
        }

        // 253-254 (Inclusive) Advanced Extended Packet Format, S-9.2.1.1
        if is_advanced_extended(raw) {
            check_advanced_extended(raw)?;

            // the framing is valid, but no advanced extended commands are handled yet
            if cfg!(feature = "verbose-transport") {
                trace!("unhandled advanced extended packet length={}", raw.len());
            }
            return Err(PacketError::InvalidAddress);
        }

        if raw.len() > MAX_EXTENDED_LEN {
            trace!("discarding packet length={}", raw.len());
            return Err(PacketError::Oversize);
        }

        if raw[0] == 0xFF {
            // idle packet
            return Ok(None);
        }

        // 192-231 (Inclusive) Multifunction Decoder Extended Address
        if is_mf_extended_address(raw) && raw.len() < 4 {
            return Err(PacketError::Undersize);
        }

        // check if the address is in the future use range
        if (232..=252).contains(&raw[0]) {
            return Err(PacketError::InvalidAddress);
        }

        if !check_xor(raw) {
            return Err(PacketError::InvalidChecksum);
        }

        // trim off the checksum byte
        let data = &raw[..raw.len() - 1];

        Ok(Some(Packet {
            data: Vec::from_slice(data).unwrap(),
        }))
    }
}

/// Checks if the frame is addressed to the advanced extended partitions (253 and 254).
fn is_advanced_extended(raw: &[u8]) -> bool {
    matches!(raw.first(), Some(253..=254))
}

/// Validates the error detection of an advanced extended packet (S-9.2.1.1 section 2.1).
///
/// Frames of up to 6 bytes carry only the XOR checksum. Longer frames carry a CRC-8 just before
/// the XOR checksum; frames of exactly 7 bytes are forbidden and must be ignored.
fn check_advanced_extended(raw: &[u8]) -> Result<(), PacketError> {
    let len = raw.len();

    // frames of exactly 7 bytes are forbidden
    if len == 7 {
        return Err(PacketError::InvalidChecksum);
    }

    // CRC over everything except the XOR byte, including the CRC itself
    if len > MAX_EXTENDED_LEN && !check_crc8(&raw[..len - 1]) {
        return Err(PacketError::InvalidChecksum);
    }

    // XOR covers every byte, including the CRC-8 when present
    if !check_xor(raw) {
        return Err(PacketError::InvalidChecksum);
    }

    Ok(())
}

/// Returns true if the XOR checksum is valid.
fn check_xor(raw: &[u8]) -> bool {
    xor(raw) == 0
}

/// XOR of all bytes. A frame with a valid error detection byte XORs to 0, because the last
/// byte is the XOR of the preceding bytes.
fn xor(raw: &[u8]) -> u8 {
    raw.iter().fold(0, |acc, &x| acc ^ x)
}

/// Returns true if the CRC-8 checksum is valid.
fn check_crc8(data: &[u8]) -> bool {
    crc8(data) == 0
}

/// CRC-8 with polynomial x^8 + x^5 + x^4 + 1 (Dallas/Maxim 1-Wire), initial value 0, not
/// inverted, as specified in S-9.2.1.1 section 2.1.
fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |mut crc, &byte| {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8C
            } else {
                crc >> 1
            };
        }
        crc
    })
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec as StdVec;

    /// Builds an advanced extended frame with the error detection the standard requires.
    fn aepf(payload: &[u8]) -> StdVec<u8> {
        let mut frame = payload.to_vec();
        if frame.len() + 1 > MAX_EXTENDED_LEN {
            frame.push(crc8(&frame));
        }
        frame.push(xor(&frame));
        frame
    }

    #[test]
    fn crc8_reference_value() {
        // Dallas/Maxim CRC-8 check value for "123456789"
        assert_eq!(crc8(b"123456789"), 0xA1);
    }

    #[test]
    fn crc8_over_frame_and_crc_is_zero() {
        let data = [0xFD, 1, 2, 3, 4, 5];
        let mut with_crc = data.to_vec();
        with_crc.push(crc8(&data));
        assert_eq!(crc8(&with_crc), 0);
    }

    #[test]
    fn advanced_extended_valid_frames_are_recognised() {
        // Get Data Start (3 bytes), Logon Enable (6 bytes), Select (13 bytes), maximum (32 bytes)
        let frames = [
            aepf(&[0xFE, 0x00]),
            aepf(&[0xFE, 0xFF, 0x12, 0x34, 0x01]),
            aepf(&[0xFE, 0xD1, 0x23, 1, 2, 3, 4, 0xFF, 5, 6, 7]),
            aepf(&[0xFD; 30]),
        ];
        for frame in frames {
            assert!(check_advanced_extended(&frame).is_ok(), "{frame:x?}");
            assert_eq!(DccCodec::decode(&frame), Err(PacketError::InvalidAddress));
        }
    }

    #[test]
    fn advanced_extended_bad_crc_is_rejected() {
        let mut frame = aepf(&[0xFD, 1, 2, 3, 4, 5, 6]);
        let crc = frame.len() - 2;
        frame[crc] ^= 0x01;
        // fix up the XOR so only the CRC is wrong
        let last = frame.len() - 1;
        frame[last] ^= 0x01;
        assert_eq!(
            check_advanced_extended(&frame),
            Err(PacketError::InvalidChecksum)
        );
    }

    #[test]
    fn advanced_extended_bad_xor_is_rejected() {
        let mut frame = aepf(&[0xFD, 1, 2, 3, 4, 5, 6]);
        let last = frame.len() - 1;
        frame[last] ^= 0x01;
        assert_eq!(
            check_advanced_extended(&frame),
            Err(PacketError::InvalidChecksum)
        );
    }

    #[test]
    fn advanced_extended_seven_bytes_is_rejected() {
        // 6 data bytes + XOR, without a CRC-8: forbidden by S-9.2.1.1 section 2.1
        let mut frame = [0xFD, 1, 2, 3, 4, 5].to_vec();
        frame.push(xor(&frame));
        assert_eq!(
            check_advanced_extended(&frame),
            Err(PacketError::InvalidChecksum)
        );
    }

    #[test]
    fn long_frames_outside_advanced_extended_are_oversize() {
        let mut frame = [0x03, 1, 2, 3, 4, 5, 6].to_vec();
        frame.push(xor(&frame));
        assert_eq!(DccCodec::decode(&frame), Err(PacketError::Oversize));
    }

    #[test]
    fn baseline_packet_decodes() {
        let packet = DccCodec::decode(&[0x03, 0x3F, 0x3C]).unwrap().unwrap();
        assert_eq!(packet.data.as_slice(), &[0x03, 0x3F]);
    }

    #[test]
    fn idle_packet_decodes_to_none() {
        assert_eq!(DccCodec::decode(&[0xFF, 0x00, 0xFF]), Ok(None));
    }
}
