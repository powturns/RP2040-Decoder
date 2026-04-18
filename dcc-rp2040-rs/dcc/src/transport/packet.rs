use crate::transport::{is_basic_address, is_mf_extended_address};
use crate::{FunctionGroup, FunctionGroupFlags, read_extended_address};
use heapless::Vec;
use int_enum::IntEnum;
use motor::{Direction, SpeedStep, VelocitySetpoint};

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Error {
    /// Packet is too short
    Undersize,

    /// Packet is too long
    Oversize,

    /// Packet has an invalid checksum
    InvalidChecksum,

    /// Packet has an invalid / unsupported address
    InvalidAddress,

    /// Packet contains an invalid instruction.
    ///
    /// May be for operation or service mode.
    InvalidInstruction,
}

/// A valid DCC packet.
#[derive(Eq, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet {
    pub(crate) data: Vec<u8, 6>,
}

impl Packet {
    /// Reads the address from the packet
    pub fn addr(&self) -> Result<u16, Error> {
        if is_basic_address(self.data.as_slice()) {
            return Ok((self.data[0] & 0b01111111) as u16);
        }

        if is_mf_extended_address(self.data.as_slice()) {
            return Ok(read_extended_address(&self.data[0..2]));
        }

        if (128..=191).contains(&self.data[0]) {
            // todo: accessory decoders
        }

        Err(Error::InvalidAddress)
    }

    /// Returns `true` if the packet may be a service mode packet.
    ///
    /// We cannot know for sure it is a service mode packet because some
    /// command mode packets have the same structure.
    pub fn service_mode_candidate(&self) -> bool {
        (112..=127u8).contains(&self.data[0])
    }

    // pub fn raw_addr(&self) -> Result<&[u8], PacketError> {
    //     if is_basic_address(self.data.as_slice()) {
    //         Ok(&self.data[..1])
    //     } else if is_mf_extended_address(self.data.as_slice()) {
    //         Ok(&self.data[..2])
    //     } else {
    //         Err(PacketError::InvalidAddress)
    //     }
    // }

    /// Returns the length of the address in bytes.
    fn address_length(&self) -> Result<usize, Error> {
        if is_basic_address(self.data.as_slice()) {
            Ok(1)
        } else if is_mf_extended_address(self.data.as_slice()) {
            Ok(2)
        } else {
            Err(Error::InvalidAddress)
        }
    }

    /// Returns the data following the address.
    fn raw_instruction_data(&self) -> Result<&[u8], Error> {
        Ok(&self.data[self.address_length()?..])
    }

    /// Returns `true` if the packet is a reset packet.
    pub fn is_reset(&self) -> bool {
        self.data.len() >= 2 && self.data[0] == 0x00 && self.data[1] == 0x00
    }
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, IntEnum)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum OperationInstructionType {
    DecoderControl = 0b000,
    AdvancedOperations = 0b001,
    SpeedDirectionReverse = 0b010,
    SpeedDirectionForward = 0b011,
    FunctionGroup1 = 0b100,
    FunctionGroup2 = 0b101,
    // Reserved = 0b110, // FeatureExpansion
    // Reserved = 0b111, // CVAccess
    FeatureExpansion = 0b110,
    CVAccess = 0b111,
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum OperationModeInstruction {
    AdvancedOperations(AdvancedOperationsInstruction),
    FunctionGroup(FunctionGroup),
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum AdvancedOperationsInstruction {
    SpeedStepControl(VelocitySetpoint),
}

impl<'a> TryFrom<&'a Packet> for OperationModeInstruction {
    type Error = Error;

    fn try_from(value: &'a Packet) -> Result<Self, Self::Error> {
        let data = &value.data[value.address_length()?..];

        let first = *data.first().ok_or(Error::Undersize)?;

        match OperationInstructionType::try_from(first >> 5)
            .map_err(|_| Error::InvalidInstruction)?
        {
            OperationInstructionType::DecoderControl => Err(Error::InvalidInstruction),
            OperationInstructionType::AdvancedOperations => {
                match first & 0b00011111 {
                    // 128 Speed Step control
                    0b11111 => {
                        let speed_vector = data.get(1).ok_or(Error::Undersize)?;

                        let direction = match (speed_vector & 0b10000000) >> 7 {
                            0 => Direction::Reverse,
                            1 => Direction::Forward,
                            _ => return Err(Error::InvalidInstruction),
                        };

                        // mask out the direction
                        let speed = speed_vector & 0b01111111;
                        let speed_step = match speed {
                            0 => SpeedStep::Stop,
                            0b0000001 => SpeedStep::EmergencyStop,
                            _ => SpeedStep::Num(speed - 1),
                        };

                        Ok(OperationModeInstruction::AdvancedOperations(
                            AdvancedOperationsInstruction::SpeedStepControl(VelocitySetpoint {
                                speed_step,
                                direction,
                            }),
                        ))
                    }
                    _ => Err(Error::InvalidInstruction),
                }
            }
            OperationInstructionType::SpeedDirectionReverse => Err(Error::InvalidInstruction),
            OperationInstructionType::SpeedDirectionForward => Err(Error::InvalidInstruction),
            OperationInstructionType::FunctionGroup1 => {
                Ok(OperationModeInstruction::FunctionGroup(parse_fg1(first)))
            }
            OperationInstructionType::FunctionGroup2 => {
                Ok(OperationModeInstruction::FunctionGroup(parse_fg2(first)))
            }
            OperationInstructionType::FeatureExpansion => Err(Error::InvalidInstruction),
            OperationInstructionType::CVAccess => Err(Error::InvalidInstruction),
        }
    }
}

fn parse_fg1(instruction: u8) -> FunctionGroup {
    let mut flags = FunctionGroupFlags::empty();

    // Group 1: 100 D D D D D (FL F4 F3 F2 F1)
    // Bits 0-3 control F1-F4, bit 4 controls FL
    if (instruction & (1 << 0)) != 0 {
        flags |= FunctionGroupFlags::F1;
    }
    if (instruction & (1 << 1)) != 0 {
        flags |= FunctionGroupFlags::F2;
    }
    if (instruction & (1 << 2)) != 0 {
        flags |= FunctionGroupFlags::F3;
    }
    if (instruction & (1 << 3)) != 0 {
        flags |= FunctionGroupFlags::F4;
    }
    if (instruction & (1 << 4)) != 0 {
        flags |= FunctionGroupFlags::FL;
    }
    FunctionGroup::new(FunctionGroupFlags::FG_1, flags)
}

fn parse_fg2(instruction: u8) -> FunctionGroup {
    let mut flags = FunctionGroupFlags::empty();

    // Group 2: 101 S D D D D
    // Bit 4 (S) determines which group: 1 = F5-F8, 0 = F9-F12
    // Bits 0-3 (DDDD) contain the function states
    let s_bit = (instruction >> 4) & 1;

    let group_mask = if s_bit == 1 {
        // F5-F8 (bits 0-3 map to F5-F8)
        if (instruction & (1 << 0)) != 0 {
            flags |= FunctionGroupFlags::F5;
        }
        if (instruction & (1 << 1)) != 0 {
            flags |= FunctionGroupFlags::F6;
        }
        if (instruction & (1 << 2)) != 0 {
            flags |= FunctionGroupFlags::F7;
        }
        if (instruction & (1 << 3)) != 0 {
            flags |= FunctionGroupFlags::F8;
        }
        FunctionGroupFlags::FG_2_1
    } else {
        // F9-F12 (bits 0-3 map to F9-F12)
        if (instruction & (1 << 0)) != 0 {
            flags |= FunctionGroupFlags::F9;
        }
        if (instruction & (1 << 1)) != 0 {
            flags |= FunctionGroupFlags::F10;
        }
        if (instruction & (1 << 2)) != 0 {
            flags |= FunctionGroupFlags::F11;
        }
        if (instruction & (1 << 3)) != 0 {
            flags |= FunctionGroupFlags::F12;
        }
        FunctionGroupFlags::FG_2_0
    };

    FunctionGroup::new(group_mask, flags)
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, IntEnum)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum ServiceInstructionType {
    ManipulateBit = 0b10,
    VerifyByte = 0b01,
    WriteByte = 0b11,
}

/// Packet extensions for when the decoder is in service mode.
pub trait ServicePacket {
    fn instruction_type(&self) -> Result<ServiceInstructionType, Error>;

    fn cv_address(&self) -> Result<u16, Error>;

    fn cv_data(&self) -> Result<u8, Error>;
}

impl ServicePacket for Packet {
    fn instruction_type(&self) -> Result<ServiceInstructionType, Error> {
        if self.data.is_empty() {
            return Err(Error::Undersize);
        }

        // Per NMRA DCC service mode (S-9.2.3), bits 4..3 of the first byte select the instruction.
        let code = (self.data[0] >> 2) & 0b11;
        code.try_into().map_err(|_| {
            trace!("invalid service mode instruction: code={:08b}", code);

            Error::InvalidInstruction
        })
    }

    fn cv_address(&self) -> Result<u16, Error> {
        if self.data.len() < 2 {
            return Err(Error::Undersize);
        }

        let msb = ((self.data[0] & 0b0000011) as u16) << 8;

        Ok(msb + self.data[1] as u16 + 1)
    }

    fn cv_data(&self) -> Result<u8, Error> {
        if self.data.len() < 3 {
            return Err(Error::Undersize);
        }

        Ok(self.data[2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::pkt;
    use crate::transport::packet::AdvancedOperationsInstruction::SpeedStepControl;

    #[test]
    fn function_group_1_instruction() {
        // Group 1: 100 D D D D D (FL F4 F3 F2 F1)
        // Example: 100 1 0 0 1 1 -> FL=1, F4=0, F3=0, F2=1, F1=1
        // Byte = 0b10010011 = 0x93
        // Address 3
        let pkt_data = [3, 0x93, 0x90]; // Checksum 3 ^ 0x93 = 0x90
        let p = pkt(&pkt_data);

        let op_instr = OperationModeInstruction::try_from(&p).unwrap();

        if let OperationModeInstruction::FunctionGroup(fg) = op_instr {
            // Test that the correct flags are set
            assert!(
                fg.flags.contains(FunctionGroupFlags::F1),
                "F1 should be set"
            );
            assert!(
                fg.flags.contains(FunctionGroupFlags::F2),
                "F2 should be set"
            );
            assert!(
                !fg.flags.contains(FunctionGroupFlags::F3),
                "F3 should not be set"
            );
            assert!(
                !fg.flags.contains(FunctionGroupFlags::F4),
                "F4 should not be set"
            );
            assert!(
                fg.flags.contains(FunctionGroupFlags::FL),
                "FL should be set"
            );

            // Test the group mask is correct
            assert_eq!(fg.group_mask, FunctionGroupFlags::FG_1);

            // Test the exact flag combination
            let expected_flags =
                FunctionGroupFlags::F1 | FunctionGroupFlags::F2 | FunctionGroupFlags::FL;
            assert_eq!(fg.flags, expected_flags);
        } else {
            panic!("Expected FunctionGroup");
        }
    }

    #[test]
    fn function_group_2_instruction_s1() {
        // Group 2: 101 S D D D D
        // S=1 -> F5-F8
        // 101 1 F8 F7 F6 F5
        // Example: 101 1 0 1 0 1 -> F8=0, F7=1, F6=0, F5=1
        // Byte = 0b10110101 = 0xB5
        // Address 3
        let pkt_data = [3, 0xB5, 0xB6]; // Checksum 3 ^ 0xB5 = 0xB6
        let p = pkt(&pkt_data);

        let op_instr = OperationModeInstruction::try_from(&p).unwrap();

        if let OperationModeInstruction::FunctionGroup(fg) = op_instr {
            // Test that the correct flags are set
            assert!(
                fg.flags.contains(FunctionGroupFlags::F5),
                "F5 should be set"
            );
            assert!(
                !fg.flags.contains(FunctionGroupFlags::F6),
                "F6 should not be set"
            );
            assert!(
                fg.flags.contains(FunctionGroupFlags::F7),
                "F7 should be set"
            );
            assert!(
                !fg.flags.contains(FunctionGroupFlags::F8),
                "F8 should not be set"
            );

            // Test the group mask is correct
            assert_eq!(fg.group_mask, FunctionGroupFlags::FG_2_1);

            // Test the exact flag combination
            let expected_flags = FunctionGroupFlags::F5 | FunctionGroupFlags::F7;
            assert_eq!(fg.flags, expected_flags);
        } else {
            panic!("Expected FunctionGroup");
        }
    }

    #[test]
    fn function_group_2_instruction_s0() {
        // Group 2: 101 S D D D D
        // S=0 -> F9-F12
        // 101 0 F12 F11 F10 F9
        // Example: 101 0 1 1 0 0 -> F12=1, F11=1, F10=0, F9=0
        // Byte = 0b10101100 = 0xAC
        // Address 3
        let pkt_data = [3, 0xAC, 0xAF]; // Checksum 3 ^ 0xAC = 0xAF
        let p = pkt(&pkt_data);

        let op_instr = OperationModeInstruction::try_from(&p).unwrap();

        if let OperationModeInstruction::FunctionGroup(fg) = op_instr {
            // Test that the correct flags are set
            assert!(
                !fg.flags.contains(FunctionGroupFlags::F9),
                "F9 should not be set"
            );
            assert!(
                !fg.flags.contains(FunctionGroupFlags::F10),
                "F10 should not be set"
            );
            assert!(
                fg.flags.contains(FunctionGroupFlags::F11),
                "F11 should be set"
            );
            assert!(
                fg.flags.contains(FunctionGroupFlags::F12),
                "F12 should be set"
            );

            // Test the group mask is correct
            assert_eq!(fg.group_mask, FunctionGroupFlags::FG_2_0);

            // Test the exact flag combination
            let expected_flags = FunctionGroupFlags::F11 | FunctionGroupFlags::F12;
            assert_eq!(fg.flags, expected_flags);
        } else {
            panic!("Expected FunctionGroup");
        }
    }

    // The following tests are based on NMRA DCC standards (S-9.2 and S-9.2.3):
    // - Service mode instruction bytes reside in 0x70..=0x7F (0b0111xxxx)
    // - Bits 3-4 of the first byte select the service-programming instruction:
    //     0b01 = Verify Byte, 0b10 = Manipulate Bit, 0b11 = Write Byte
    // - CV address is formed from the low two bits of the first byte (MSBs)
    //   concatenated with the second byte (LSBs), yielding a 10-bit CV number
    //   in the range 0..=1023. The third byte is the data byte.

    #[test]
    fn service_mode_candidate_range() {
        // 0x70..=0x7F should be detected as service-mode candidates
        for b in 0x70u8..=0x7F {
            assert!(
                pkt(&[b]).service_mode_candidate(),
                "0x{b:02X} not detected as service mode candidate"
            );
        }

        // Some bytes outside the range should not be service-mode candidates
        for &b in &[0x00u8, 0x10, 0x6F, 0x80, 0xFF] {
            assert!(
                !pkt(&[b]).service_mode_candidate(),
                "0x{b:02X} incorrectly detected as service mode candidate"
            );
        }
    }

    #[test]
    fn service_instruction_type_decode() {
        let verify = pkt(&[0b0100, 0x00, 0x00]);
        let manipulate = pkt(&[0b1000, 0x00, 0x00]);
        let write = pkt(&[0b1100, 0x00, 0x00]);

        assert_eq!(
            verify.instruction_type().unwrap(),
            ServiceInstructionType::VerifyByte
        );
        assert_eq!(
            manipulate.instruction_type().unwrap(),
            ServiceInstructionType::ManipulateBit
        );
        assert_eq!(
            write.instruction_type().unwrap(),
            ServiceInstructionType::WriteByte
        );
    }

    #[test]
    fn service_mode_cv_address_and_data() {
        let p1 = pkt(&[0b0111_0100, 0x24, 0x99]);
        assert_eq!(p1.cv_address().unwrap(), 0x025);
        assert_eq!(p1.cv_data().unwrap(), 0x99);

        // Example 2: MSBs = 0b11, LSBs = 0xFF -> CV = 0x3FF (1023, max per standard)
        let p2 = pkt(&[0b0111_0011, 0xFE, 0x55]);
        assert_eq!(p2.cv_address().unwrap(), 0x3FF);
        assert_eq!(p2.cv_data().unwrap(), 0x55);
    }

    #[test]
    fn service_packet_undersize_errors() {
        // Empty packet
        let p = pkt(&[]);
        assert_eq!(p.instruction_type().unwrap_err(), Error::Undersize);

        // Only one byte -> instruction_type ok, but cv_address needs 2 bytes
        let p = pkt(&[0x70]);
        assert_eq!(p.cv_address().unwrap_err(), Error::Undersize);

        // Two bytes -> cv_data needs 3 bytes
        let p = pkt(&[0x70, 0x00]);
        assert_eq!(p.cv_data().unwrap_err(), Error::Undersize);
    }

    #[test]
    fn reset_packet_detection() {
        assert!(pkt(&[0x00, 0x00]).is_reset());
        assert!(!pkt(&[0x00]).is_reset());
        assert!(!pkt(&[0x00, 0x01]).is_reset());
    }

    #[test]
    fn operation_mode_instruction_128_speed_control() {
        let directions = [Direction::Forward, Direction::Reverse];

        for &direction in directions.iter() {
            let dir = match direction {
                Direction::Forward => 1,
                Direction::Reverse => 0,
            } << 7;

            assert_eq!(
                Ok(OperationModeInstruction::AdvancedOperations(
                    SpeedStepControl(VelocitySetpoint::new(SpeedStep::Stop, direction))
                )),
                OperationModeInstruction::try_from(&pkt(&[3, 0b00111111, 0b00000000 + dir]))
            );

            assert_eq!(
                Ok(OperationModeInstruction::AdvancedOperations(
                    SpeedStepControl(VelocitySetpoint::new(SpeedStep::EmergencyStop, direction))
                )),
                OperationModeInstruction::try_from(&pkt(&[3, 0b00111111, 0b00000001 + dir]))
            );

            assert_eq!(
                Ok(OperationModeInstruction::AdvancedOperations(
                    SpeedStepControl(VelocitySetpoint::new(SpeedStep::Num(1), direction))
                )),
                OperationModeInstruction::try_from(&pkt(&[3, 0b00111111, 0b00000010 + dir]))
            );

            assert_eq!(
                Ok(OperationModeInstruction::AdvancedOperations(
                    SpeedStepControl(VelocitySetpoint::new(SpeedStep::Num(126), direction))
                )),
                OperationModeInstruction::try_from(&pkt(&[3, 0b00111111, 0b01111111 + dir]))
            );
        }
    }
}
