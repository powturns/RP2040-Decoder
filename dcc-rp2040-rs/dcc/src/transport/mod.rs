use crate::transport::packet::{Error as PacketError, Packet};
use codec::DccCodec;

pub mod codec;
pub mod packet;

pub struct Decoder<D: RawDccDecoder> {
    inner: D,
}

impl<D: RawDccDecoder> Decoder<D> {
    pub fn new(inner: D) -> Self {
        Self { inner }
    }

    pub async fn read(&mut self) -> Packet {
        let mut buff = [0u32; 2];
        let aligned_buff = bytemuck::bytes_of_mut(&mut buff);

        loop {
            let data = self.inner.read(aligned_buff).await;

            match DccCodec::decode(data) {
                Ok(p) => match p {
                    None => {
                        if cfg!(feature = "verbose-transport") {
                            trace!("idle packet")
                        }
                    }
                    Some(p) => return p,
                },
                Err(e) => {
                    debug!("invalid packet: {:?}", e);
                }
            }
        }
    }
}

/// Implementors of this trait can read raw DCC packets.
pub trait RawDccDecoder {
    /// Read a DCC packet using the specified buffer.
    ///
    /// The buffer must be at least 8 bytes long and aligned to a 4 byte boundary.
    fn read<'a>(&mut self, buff: &'a mut [u8]) -> impl Future<Output = &'a [u8]>;
}

/// Checks if the given raw packet contains a basic (one byte) address.
fn is_basic_address(raw_packet: &[u8]) -> bool {
    (0..=127).contains(&raw_packet[0])
}

/// Checks if the given raw packet contains an extended (two byte) address for a
/// multifunction decoder.
fn is_mf_extended_address(raw_packet: &[u8]) -> bool {
    if raw_packet.len() < 2 {
        return false;
    }

    (192..=231).contains(&raw_packet[0])
}

#[cfg(test)]
mod tests {
    #[cfg(test)]
    extern crate std;
    use super::*;

    /// Helper function to create a Packet directly from raw bytes
    fn create_packet(bytes: &[u8]) -> Packet {
        Packet {
            data: Vec::from_slice(bytes).unwrap(),
        }
    }

    #[test]
    fn test_standard_addresses() {
        // Test standard 7-bit addresses (0-127)
        // Uses bits 0-6 of first byte

        // Address 0 (broadcast)
        let packet = create_packet(&[0x00, 0x60, 0x60]);
        assert_eq!(packet.addr(), Ok(0));

        // Address 1
        let packet = create_packet(&[0x01, 0x60, 0x61]);
        assert_eq!(packet.addr(), Ok(1));

        // Address 3 (common default)
        let packet = create_packet(&[0x03, 0x60, 0x63]);
        assert_eq!(packet.addr(), Ok(3));

        // Address 64 (middle range)
        let packet = create_packet(&[0x40, 0x60, 0x20]);
        assert_eq!(packet.addr(), Ok(64));

        // Address 127 (maximum standard - all 7 bits set)
        let packet = create_packet(&[0x7F, 0x60, 0x1F]);
        assert_eq!(packet.addr(), Ok(127));
    }

    #[test]
    fn test_basic_accessory_decoder_addresses() {
        // Test Basic Accessory Decoder addresses (128-191)
        // These are not implemented yet, so should return InvalidAddress

        // Address 128
        let packet = create_packet(&[128, 0x80, 0x00]);
        assert_eq!(packet.addr(), Err(PacketError::InvalidAddress));

        // Address 150
        let packet = create_packet(&[150, 0x80, 0x96]);
        assert_eq!(packet.addr(), Err(PacketError::InvalidAddress));

        // Address 191 (maximum basic accessory)
        let packet = create_packet(&[191, 0x80, 0x31]);
        assert_eq!(packet.addr(), Err(PacketError::InvalidAddress));
    }

    #[test]
    fn test_extended_accessory_decoder_addresses() {
        // Test Extended Accessory Decoder addresses (128-191)
        // These are not implemented yet, so should return InvalidAddress

        // Address 128 in longer packet
        let packet = create_packet(&[128, 0x01, 0x80, 0x49]);
        assert_eq!(packet.addr(), Err(PacketError::InvalidAddress));

        // Address 160 in longer packet
        let packet = create_packet(&[160, 0x02, 0x80, 0x22]);
        assert_eq!(packet.addr(), Err(PacketError::InvalidAddress));
    }

    #[test]
    fn test_multifunction_decoder_extended_addresses() {
        // Test Multifunction Decoder Extended addresses (192-231)
        // These use 14-bit addressing: (first_byte & 0x3F) << 8 | second_byte

        let packet = create_packet(&[192, 0x00, 0x60, 0xF2]);
        assert_eq!(packet.addr(), Ok(0));

        let packet = create_packet(&[192, 0x01, 0x60, 0xF3]);
        assert_eq!(packet.addr(), Ok(1));

        let packet = create_packet(&[192, 0xFF, 0x60, 0x0D]);
        assert_eq!(packet.addr(), Ok(255));

        let packet = create_packet(&[193, 0x00, 0x60, 0xF1]);
        assert_eq!(packet.addr(), Ok(256));

        let packet = create_packet(&[195, 0x2C, 0x60, 0xDF]);
        assert_eq!(packet.addr(), Ok(812));

        let packet = create_packet(&[200, 0x00, 0x60, 0xE8]);
        assert_eq!(packet.addr(), Ok(2048));

        let packet = create_packet(&[231, 0xFF, 0x60, 0x76]);
        assert_eq!(packet.addr(), Ok(10239));
    }

    #[test]
    fn test_address_boundary_conditions() {
        // Test boundary between standard and extended addressing

        // Address 191 (accessory decoder - not implemented)
        let packet = create_packet(&[191, 0x80, 0x31]);
        assert_eq!(packet.addr(), Err(PacketError::InvalidAddress));

        // Address 192 (first extended range)
        let packet = create_packet(&[192, 0x80, 0x60, 0x72]);
        assert_eq!(packet.addr(), Ok(128));

        // Address 224 (extended range)
        let packet = create_packet(&[224, 0x00, 0x60, 0xC4]);
        assert_eq!(packet.addr(), Ok(8192));
    }

    #[test]
    fn test_extended_address_detection() {
        // Verify is_extended_address function works correctly
        assert!(!is_mf_extended_address(&[127])); // Standard address
        assert!(!is_mf_extended_address(&[191])); // Accessory decoder
        assert!(is_mf_extended_address(&[192, 0])); // Extended address
        assert!(is_mf_extended_address(&[200, 0])); // Extended address
        assert!(is_mf_extended_address(&[231, 0])); // Extended address
        assert!(!is_mf_extended_address(&[232])); // Future use
        assert!(!is_mf_extended_address(&[])); // Empty packet
        assert!(!is_mf_extended_address(&[192])); // Single byte (length < 2)
    }
}
