use crate::transport::{Packet, PacketError, is_mf_extended_address};
use heapless::Vec;

pub(crate) struct DccCodec {}

impl DccCodec {
    /// Attempts to decode the packet.
    pub fn decode(raw: &[u8]) -> Result<Option<Packet>, PacketError> {
        // todo: if we ever want to support advanced extended packets we'll need a way to accumulate the bytes
        //       until we get a full packet. This also means we'll probably want some variable sized Packet struct
        //       to minimize the amount of data we move around

        if raw.len() > 6 {
            trace!("discarding packet length={}", raw.len());
            return Err(PacketError::Oversize);
        }

        // packets must be at least 3 bytes
        if raw.len() < 3 {
            return Err(PacketError::Undersize);
        }

        if raw[0] == 0xFF {
            // idle packet
            return Ok(None);
        }

        // 192-231 (Inclusive) Multifunction Decoder Extended Address
        if is_mf_extended_address(raw) && raw.len() < 4 {
            return Err(PacketError::Undersize);
        }

        // check if the address is in the future use range / advanced extended packet format
        if (232..=254).contains(&raw[0]) {
            return Err(PacketError::InvalidAddress);
        }

        // after XORing all the bytes, we should be left with 0
        // this is because the last checksum byte is the XOR of the previous bytes,
        // so XORing it with the previous bytes will result in 0
        let checksum = raw.iter().fold(0, |acc, &x| acc ^ x);

        if checksum != 0 {
            return Err(PacketError::InvalidChecksum);
        }

        // trim off the checksum byte
        let data = &raw[..raw.len() - 1];

        Ok(Some(Packet {
            data: Vec::from_slice(data).unwrap(),
        }))
    }
}
