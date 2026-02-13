use crate::cv::store::{reset, Error as StoreError, StoreExt};
use crate::cv::store::Store;
use crate::handler::Op::{AcknowledgeCv, Reboot};
use crate::transport::packet::{
    AdvancedOperationsInstruction, Error as PacketError, OperationModeInstruction, Packet,
    ServiceInstructionType, ServicePacket,
};
use crate::{FunctionGroupFlags, Timer, is_recipient, FunctionGroup, is_broadcast};
use motor::VelocitySetpoint;
use crate::cv::Cv;

const SERVICE_MODE_TIMEOUT: usize = 20;

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Op {
    /// Acknowledge the CV has been written using the service mode technique.
    AcknowledgeCv,

    /// Stops the decoder operations and returns it to the startup state.
    Reset,

    /// A 128 speed step velocity setpoint.
    Velocity128(VelocitySetpoint),

    /// Request to set a function group state.
    SetFunctions(FunctionGroup),

    /// Reboots the decoder.
    Reboot,
}

/// Contains the logic for handling packets.
pub struct Handler<T, S>
where
    T: Timer,
    S: Store,
{
    enter_service_mode_timer: T,
    store: S,
}

impl<T, S> Handler<T, S>
where
    T: Timer,
    S: Store,
{
    pub fn new(timer: T, store: S) -> Self {
        Self { enter_service_mode_timer: timer, store }
    }

    /// Handles the packet, returning any operation that needs to be performed.
    pub fn handle(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        if packet.is_reset() {
            // we may be entering service mode
            self.enter_service_mode_timer.start();
            self.handle_reset(packet).map(Some)
        } else if let Some(elapsed) = self.enter_service_mode_timer.elapsed()
            && elapsed < SERVICE_MODE_TIMEOUT
            && packet.service_mode_candidate()
        {
            self.enter_service_mode_timer.start();

            self.handle_service_mode(packet)
        } else if is_recipient(packet, &self.store) {
            // this packet was specifically addressed to us (not a broadcast)
            self.enter_service_mode_timer.stop();

            self.handle_command(packet)
        } else if is_broadcast(packet) {
            // TODO: should we handle packets that are broadcast?
            self.handle_command(packet)
        } else {
            // packet not addressed to us
            Ok(None)
        }
    }

    fn handle_command(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        match OperationModeInstruction::try_from(packet)? {
            OperationModeInstruction::AdvancedOperations(o) => match o {
                AdvancedOperationsInstruction::SpeedStepControl(v) => Ok(Some(Op::Velocity128(v))),
            },
            OperationModeInstruction::FunctionGroup(fg) => {
                Ok(Some(Op::SetFunctions(fg)))
            }
        }
    }

    fn handle_service_mode(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        let packet_type = packet.instruction_type()?;
        trace!("handle_service_mode: type={:?}, packet={:08b}", packet_type, packet.data);
        match packet_type {
            ServiceInstructionType::ManipulateBit => self.manipulate_bit(packet),
            ServiceInstructionType::VerifyByte => self.verify_byte(packet),
            ServiceInstructionType::WriteByte => self.write_byte(packet),
        }
    }

    fn handle_reset(&mut self, packet: &Packet) -> Result<Op, Error> {
        Ok(Op::Reset)
    }

    fn verify_byte(&self, packet: &Packet) -> Result<Option<Op>, Error> {
        let expected = packet.cv_data()?;
        let actual = self.store.read_byte(packet.cv_address()? as usize)?;

        let op = if expected == actual {
            Some(Op::AcknowledgeCv)
        } else {
            None
        };

        Ok(op)
    }

    fn write_byte(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        // TODO: we dont allow writing every cv!, some are reserved for special operations!

        let address = packet.cv_address()?;

        let write = match Cv::try_from(address) {
            Ok(addr) => match addr {

                // read only cvs
                Cv::ManufacturerId => {
                    // Reset all CVs to default when setting CV_8 = 8)
                    if packet.cv_data()? == 8 {
                        info!("Resetting all CVs to default");
                        reset(&mut self.store)?;
                    }

                    return Ok(Some(Reboot));
                }

                Cv::ManufacturerVersionNumber => {
                    // ADC offset Adjustment is triggered when setting CV_7 = 7
                    if packet.cv_data()? == 7 {
                        self.store.emf_adc_offset_clear()?;
                    }

                    return Ok(Some(Reboot));
                }

                _ => true
            }

            _ => true
        };

        if write {
            self.store
                .write_byte(address as usize, packet.cv_data()?)?;
        }

        Ok(Some(AcknowledgeCv))
    }

    fn manipulate_bit(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
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
        let current = self.store.read_byte(cv_addr)?;
        let mask = 1u8 << bit_pos;

        if is_write {
            // Write Bit operation - branchless bit manipulation
            let new_val = (current & !mask) | (d_val << bit_pos);
            self.store.write_byte(cv_addr, new_val)?;
            Ok(Some(AcknowledgeCv))
        } else {
            // Bit Verify operation - direct XOR comparison
            let bit_matches = ((current >> bit_pos) & 1) == d_val;
            Ok(if bit_matches {
                Some(AcknowledgeCv)
            } else {
                None
            })
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cv::Cv::{DecoderConfiguration, ExtendedAddressMsb, PrimaryAddress};
    use crate::handler::Op::Reset;
    use crate::testing::{MockStore, MockTimer, pkt};
    use motor::{Direction, SpeedStep};

    fn reset_packet() -> Packet {
        pkt(&[0x00, 0x00, 0x00])
    }

    fn service_verify_packet(cv_addr: u16, value: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        pkt(&[0x70 | (0b01 << 3) | addr_high, addr_low, value, 0x00])
    }

    fn service_write_packet(cv_addr: u16, value: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        pkt(&[0x70 | (0b11 << 3) | addr_high, addr_low, value, 0x00])
    }

    fn service_bit_verify_packet(cv_addr: u16, bit_pos: u8, bit_val: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        let data = 0b1110_0000 | (bit_val << 3) | bit_pos;
        pkt(&[0x70 | (0b10 << 3) | addr_high, addr_low, data, 0x00])
    }

    fn service_bit_write_packet(cv_addr: u16, bit_pos: u8, bit_val: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        let data = 0b1111_0000 | (bit_val << 3) | bit_pos;
        pkt(&[0x70 | (0b10 << 3) | addr_high, addr_low, data, 0x00])
    }

    fn operation_packet(addr: u16, instruction: &[u8]) -> Packet {
        let mut vec = alloc::vec::Vec::new();
        if addr <= 127 {
            vec.push(addr as u8);
        } else {
            vec.push(0xC0 | ((addr >> 8) as u8)); // address high byte
            vec.push((addr & 0xFF) as u8); // address low byte
        }

        vec.extend_from_slice(instruction);

        pkt(&vec)
    }

    fn advanced_speed_step_packet(addr: u16, speed_step: SpeedStep) -> Packet {
        let instruction = match speed_step {
            SpeedStep::Stop => 0b00000000,
            SpeedStep::EmergencyStop => 0b00000001,
            SpeedStep::Num(n) => n + 1, // add one to account for emergency stop being 1
        };
        operation_packet(addr, &[0b00111111, instruction])
    }

    // Test harness to reduce boilerplate
    struct TestHarness {
        handler: Handler<MockTimer, MockStore>,
    }

    impl TestHarness {
        fn new() -> Self {
            Self {
                handler: Handler::new(MockTimer::new(), MockStore::new()),
            }
        }

        fn with_address(addr: u8) -> Self {
            Self {
                handler: Handler::new(
                    MockTimer::new(),
                    MockStore::new().with_cv(PrimaryAddress as u16, addr),
                ),
            }
        }

        fn with_extended_address(addr: u16) -> Self {
            let cv_base = ExtendedAddressMsb as u16;

            Self {
                handler: Handler::new(
                    MockTimer::new(),
                    MockStore::new()
                        .with_cv(
                            // address high byte
                            cv_base,
                            (addr >> 8) as u8,
                        )
                        .with_cv(
                            // address low byte
                            cv_base + 1,
                            addr as u8,
                        )
                        .with_cv(DecoderConfiguration as u16, 0b00100000), // put decoder into extended address mode
                ),
            }
        }

        fn with_store(store: MockStore) -> Self {
            Self {
                handler: Handler::new(MockTimer::new(), store),
            }
        }

        fn handle(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
            self.handler.handle(packet)
        }

        fn set_timer_elapsed(&mut self, elapsed: usize) {
            self.handler.enter_service_mode_timer.set_elapsed(elapsed);
        }

        fn start_timer(&mut self) {
            self.handler.enter_service_mode_timer.start();
        }

        fn enter_service_mode(&mut self) {
            self.start_timer();
            self.set_timer_elapsed(10);
        }

        fn read_cv(&self, addr: usize) -> u8 {
            self.handler.store.read_byte(addr).expect("error reading cv")
        }
    }

    #[test]
    fn test_reset_packet_starts_timer() {
        let mut harness = TestHarness::new();
        let result = harness.handle(&reset_packet());

        assert_eq!(result, Ok(Some(Reset)));
        assert!(harness.handler.enter_service_mode_timer.running);
    }

    #[test]
    fn test_service_verify_byte_match() {
        let store = MockStore::new().with_cv(10, 0x42);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let packet = service_verify_packet(10, 0x42);
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
    }

    #[test]
    fn test_service_verify_byte_mismatch() {
        let store = MockStore::new().with_cv(10, 0x42);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let packet = service_verify_packet(10, 0x99);
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
    }

    #[test]
    fn test_service_write_byte() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let packet = service_write_packet(20, 0xAB);
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(20), 0xAB);
    }

    #[test]
    fn test_service_bit_verify_match() {
        let store = MockStore::new().with_cv(
            15,
            0b0000_1000, // bit 3 is set
        );
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let packet = service_bit_verify_packet(15, 3, 1);
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
    }

    #[test]
    fn test_service_bit_verify_mismatch() {
        let store = MockStore::new().with_cv(
            15,
            0b0000_1000, // bit 3 is set
        );
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let packet = service_bit_verify_packet(15, 3, 0);
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(None));
    }

    #[test]
    fn test_service_bit_write_set() {
        let store = MockStore::new().with_cv(
            25,
            0b0000_0000, // bit 3 is set
        );
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_bit_write_packet(25, 5, 1); // set bit 5
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(25), 0b0010_0000);
    }

    #[test]
    fn test_service_bit_write_clear() {
        let store = MockStore::new().with_cv(25, 0xFF);
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_bit_write_packet(25, 2, 0); // clear bit 2
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(25), 0b1111_1011);
    }

    #[test]
    fn test_service_mode_timeout() {
        let mut harness = TestHarness::new();

        harness.start_timer();
        harness.set_timer_elapsed(SERVICE_MODE_TIMEOUT + 1); // past timeout

        let packet = service_verify_packet(10, 0x42);
        let result = harness.handle(&packet);

        // Should not be handled as service mode packet
        assert_eq!(result, Ok(None));
    }

    #[test]
    fn test_operation_mode_basic_address() {
        let mut harness = TestHarness::with_address(3);

        let packet = advanced_speed_step_packet(3, SpeedStep::Num(50));
        let result = harness.handle(&packet);

        assert_eq!(
            result,
            Ok(Some(Op::Velocity128(VelocitySetpoint {
                speed_step: SpeedStep::Num(50),
                direction: Direction::Reverse,
            })))
        );
    }

    #[test]
    fn test_operation_mode_extended_address() {
        let mut harness = TestHarness::with_extended_address(5000);

        let packet = advanced_speed_step_packet(5000, SpeedStep::Num(100));
        let result = harness.handle(&packet);

        assert_eq!(
            result,
            Ok(Some(Op::Velocity128(VelocitySetpoint {
                speed_step: SpeedStep::Num(100),
                direction: Direction::Reverse,
            })))
        );
    }

    #[test]
    fn test_operation_mode_stop() {
        let mut harness = TestHarness::with_address(10);

        let packet = advanced_speed_step_packet(10, SpeedStep::Stop);
        let result = harness.handle(&packet);

        assert_eq!(
            result,
            Ok(Some(Op::Velocity128(VelocitySetpoint {
                speed_step: SpeedStep::Stop,
                direction: Direction::Reverse,
            })))
        );
    }

    #[test]
    fn test_operation_mode_emergency_stop() {
        let mut harness = TestHarness::with_address(10);

        let packet = advanced_speed_step_packet(10, SpeedStep::EmergencyStop);
        let result = harness.handle(&packet);

        assert_eq!(
            result,
            Ok(Some(Op::Velocity128(VelocitySetpoint {
                speed_step: SpeedStep::EmergencyStop,
                direction: Direction::Reverse,
            })))
        );
    }

    #[test]
    fn test_packet_not_for_us() {
        let mut harness = TestHarness::with_address(3);

        // Send packet addressed to different decoder
        let packet = advanced_speed_step_packet(99, SpeedStep::Num(50));
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(None));
    }

    #[test]
    fn test_service_mode_sequence() {
        let mut harness = TestHarness::with_address(3);

        // Start with reset
        let result = harness.handle(&reset_packet());
        assert_eq!(result, Ok(Some(Op::Reset)));

        // Timer should be running, set elapsed time
        harness.set_timer_elapsed(5);

        // Write CV
        let result = harness.handle(&service_write_packet(100, 0x55));
        assert_eq!(result, Ok(Some(Op::AcknowledgeCv)));
        assert_eq!(harness.read_cv(100), 0x55);

        // Verify CV
        let result = harness.handle(&service_verify_packet(100, 0x55));
        assert_eq!(result, Ok(Some(Op::AcknowledgeCv)));
    }

    #[test]
    fn test_bit_manipulation_all_positions() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        // Test writing each bit position
        for bit_pos in 0..8 {
            let packet = service_bit_write_packet(50, bit_pos, 1);
            let result = harness.handle(&packet);
            assert_eq!(result, Ok(Some(Op::AcknowledgeCv)));
        }

        // All bits should be set
        assert_eq!(harness.read_cv(50), 0xFF);

        // Clear each bit
        for bit_pos in 0..8 {
            let packet = service_bit_write_packet(50, bit_pos, 0);
            let result = harness.handle(&packet);
            assert_eq!(result, Ok(Some(Op::AcknowledgeCv)));
        }

        // All bits should be clear
        assert_eq!(harness.read_cv(50), 0x00);
    }
}
