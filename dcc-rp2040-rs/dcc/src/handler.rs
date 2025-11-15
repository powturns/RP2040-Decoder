use crate::cv::store::Store;
use crate::{is_recipient, Timer};
use crate::transport::{Packet, PacketError};
use crate::cv::store::Error as StoreError;
use crate::handler::Op::AcknowledgeCv;

const SERVICE_MODE_TIMEOUT: usize = 20;

pub enum Op {
    AcknowledgeCv,
}

/// Contains the logic for handling packets.
pub struct Handler<T, S>
where T: Timer,
      S: Store,{
    timer:T,
    store: S,
}

impl <T, S> Handler<T, S>
where T: Timer, S: Store {
    pub fn new(
        timer: T,
        store: S
    ) -> Self {
        Self {
            timer,
            store
        }
    }

    /// Handles the packet, returning any operation that needs to be performed.
    pub fn handle(&mut self, packet: Packet)-> Result<Option<Op>, Error> {
        if packet.is_reset() {
            // we may be entering service mode
            self.timer.start();
            self.handle_reset(packet).map(|opt| Some(opt))
        } else if let Some(elapsed) = self.timer.elapsed() && elapsed < SERVICE_MODE_TIMEOUT && packet.service_mode_candidate() {
            self.timer.start();

            self.handle_service_mode(packet)
        } else if is_recipient(&packet, &self.store) {
            // this packet was specifically addressed to us (not a broadcast)
            self.timer.stop();

            self.handle_command(packet)
        } else {
            // packet not addressed to us
            Ok(None)
        }
    }

    fn handle_command(&mut self, packet: Packet) -> Result<Option<Op>, Error> {
        todo!()
    }

    fn handle_service_mode(&mut self, packet: Packet) -> Result<Option<Op>, Error> {
        match packet.instruction_type()? {
            ServiceModeInstruction::ManipulateBit => {
                self.manipulate_bit(packet)
            }
            ServiceModeInstruction::VerifyByte => {
                self.verify_byte(packet)
            }
            ServiceModeInstruction::WriteByte => {
                self.write_byte(packet)
            }
        }
    }

    fn verify_byte(&self, packet:Packet) -> Result<Option<Op>, Error> {
        let expected = packet.cv_data()?;
        let actual = self.store.read_cv(packet.cv_address()? as usize);

        let op = if expected == actual {
            Some(Op::AcknowledgeCv)
        } else {
            None
        };

        Ok(op)
    }

    fn write_byte(&mut self, packet:Packet) -> Result<Option<Op>, Error> {
        // TODO: we dont allow writing every cv!, some are reserved for special operations!

        self.store.write_cv(packet.cv_address()? as usize, packet.cv_data()?)?;

        Ok(Some(AcknowledgeCv))
    }

    fn manipulate_bit(&mut self, packet:Packet) -> Result<Option<Op>, Error> {
        // cv_data format: 111KDBBB

        let packed = packet.cv_data()?;

        // top 3 bits must be 111
        if packed < 0b1110_0000 {
            return Err(PacketError::InvalidInstruction.into());
        }

        // unpack the data
        // k - 1 = write, 0 = read
        // d - bit value
        // bbb - BBB represents the bit position within the CV (000 being defined as bit 0)
        let is_write = (packed & 0b0001_0000) != 0; // K bit
        let d_val = (packed & 0b0000_1000) >> 3; // D bit
        let bit_pos = packed & 0b0000_0111; // BBB

        let cv_addr = packet.cv_address()? as usize;
        let current = self.store.read_cv(cv_addr);
        let mask = 1u8 << bit_pos;

        if is_write {
            // Write Bit operation - branchless bit manipulation
            let new_val = (current & !mask) | (d_val << bit_pos);
            self.store.write_cv(cv_addr, new_val)?;
            Ok(Some(AcknowledgeCv))
        } else {
            // Bit Verify operation - direct XOR comparison
            let bit_matches = ((current >> bit_pos) & 1) == d_val;
            Ok(if bit_matches { Some(AcknowledgeCv) } else { None })
        }
    }

    fn handle_reset(&mut self, packet: Packet) -> Result<Op, Error> {
        // emergency stop the motor
        // disable functions

        todo!()
    }
}

enum ServiceModeInstruction {
    ManipulateBit,
    VerifyByte,
    WriteByte,
}

impl TryFrom<u8> for ServiceModeInstruction {
    type Error = PacketError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b10 => Ok(ServiceModeInstruction::ManipulateBit),
            0b01 => Ok(ServiceModeInstruction::VerifyByte),
            0b11 => Ok(ServiceModeInstruction::WriteByte),
            _ => Err(PacketError::InvalidInstruction),
        }
    }
}

/// Packet extensions for when the decoder is in service mode.
trait ServicePacket {
    fn instruction_type(&self) -> Result<ServiceModeInstruction, PacketError>;

    fn cv_address(&self) -> Result<u16, PacketError>;

    fn cv_data(&self) -> Result<u8, PacketError>;
}

impl ServicePacket for Packet {
    fn instruction_type(&self) -> Result<ServiceModeInstruction, PacketError> {
        if self.data.is_empty() {
            return Err(PacketError::Undersize)
        }

        (self.data[0] >> 3).try_into()
    }

    fn cv_address(&self) -> Result<u16, PacketError> {
        if self.data.len() < 2 {
            return Err(PacketError::Undersize);
        }

        let msb = ((self.data[0] & 0b0000011) as u16) << 8;

        Ok(msb + self.data[1] as u16)
    }

    fn cv_data(&self) -> Result<u8, PacketError> {
        if self.data.len() < 3 {
            return Err(PacketError::Undersize);
        }

        Ok(self.data[2])
    }
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Error {
    /// Problem with the packet
    Packet(PacketError),

    Store(StoreError),
}

impl From<PacketError> for Error {
    fn from(value: PacketError) -> Self {
        Self::Packet(value)
    }
}

impl From<StoreError> for Error {
    fn from(value: StoreError) -> Self {
        Self::Store(value)
    }
}