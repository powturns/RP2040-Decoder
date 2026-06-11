use crate::cv::Cv;
use crate::cv::store::Store;
use crate::cv::store::{Error as StoreError, StoreExt, reset};
use crate::handler::Op::{AcknowledgeCv, Reboot};
use crate::transport::packet::{
    AdvancedOperationsInstruction, Error as PacketError, OperationModeInstruction, Packet,
    ServiceInstructionType, ServicePacket,
};
use crate::{FunctionGroup, FunctionGroupFlags, Timer, is_broadcast, is_recipient};
use motor::VelocitySetpoint;

const SERVICE_MODE_TIMEOUT_MS: usize = 20;

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
    pending_service_packet: Option<Packet>,
}

impl<T, S> Handler<T, S>
where
    T: Timer,
    S: Store,
{
    pub fn new(timer: T, store: S) -> Self {
        Self {
            enter_service_mode_timer: timer,
            store,
            pending_service_packet: None,
        }
    }

    /// Handles the packet, returning any operation that needs to be performed.
    pub fn handle(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        if packet.is_reset() {
            // we may be entering service mode
            self.enter_service_mode_timer.start();
            self.handle_reset(packet).map(Some)
        } else if let Some(elapsed) = self.enter_service_mode_timer.elapsed_ms()
            && elapsed < SERVICE_MODE_TIMEOUT_MS
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
            OperationModeInstruction::FunctionGroup(fg) => Ok(Some(Op::SetFunctions(fg))),
        }
    }

    fn handle_service_mode(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        if self.pending_service_packet.take().as_ref() != Some(packet) {
            self.pending_service_packet = Some(packet.clone());
            return Ok(None);
        }

        // check for legacy reset packet
        if packet.data.len() == 2 && packet.data[..2] == [0b01111111, 0b00001000] {
            self.reset_to_factory_defaults()?;
            return Ok(Some(Reboot));
        }

        let packet_type = packet.instruction_type()?;
        trace!(
            "handle_service_mode: type={:?}, packet={:08b}",
            packet_type, packet.data
        );

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
        let address = packet.cv_address()?;

        // CV31/CV32 select the CV index page (S-9.2.2). This decoder does not implement
        // indexed CVs, so the writes are silently ignored (matches core0.c:write_cv_handler).
        if address == 31 || address == 32 {
            return Ok(None);
        }

        let write = match Cv::try_from(address) {
            Ok(addr) => match addr {
                Cv::PrimaryAddress => {
                    let value = packet.cv_data()?;
                    if !(1..=127).contains(&value) {
                        // return an error if the value is not in the valid primary address
                        // range
                        return Err(PacketError::InvalidInstruction.into());
                    }
                    self.store.write_byte(Cv::PrimaryAddress as usize, value)?;

                    // clear the extended address mode bit and consist addresses
                    let cv29 = self.store.read_byte(Cv::DecoderConfiguration as usize)?;
                    self.store.write_byte(Cv::DecoderConfiguration as usize, cv29 & !0b00100000)?;
                    self.store.write_byte(Cv::ConsistAddress as usize, 0)?;
                    return Ok(Some(AcknowledgeCv));
                }

                // read only cvs
                Cv::ManufacturerId => {
                    // Reset all CVs to default when setting CV_8 = 8)
                    if packet.cv_data()? == 8 {
                        self.reset_to_factory_defaults()?;
                        return Ok(Some(Reboot));
                    }
                    return Ok(None);
                }

                Cv::ManufacturerVersionNumber => {
                    // ADC offset Adjustment is triggered when setting CV_7 = 7
                    if packet.cv_data()? == 7 {
                        self.store.emf_adc_offset_clear()?;
                    }

                    return Ok(Some(Reboot));
                }

                Cv::ExtendedAddressMsb => {
                    // CV17 holds the high 6 bits of the extended address and must be in
                    // 192..=231. CV17 == 192 with CV18 == 0 would form address 0, which is
                    // invalid. (core0.c:write_cv_handler)
                    let value = packet.cv_data()?;
                    let cv18 = self.store.read_byte(Cv::ExtendedAddressLsb as usize)?;
                    if !(192..=231).contains(&value) || (cv18 == 0 && value == 192) {
                        return Err(PacketError::InvalidInstruction.into());
                    }
                    self.store.write_byte(Cv::ExtendedAddressMsb as usize, value)?;
                    return Ok(Some(AcknowledgeCv));
                }

                Cv::ExtendedAddressLsb => {
                    // CV18 holds the low 8 bits. Reject the value that would form address 0
                    // when CV17 is already at its minimum (192). (core0.c:write_cv_handler)
                    let value = packet.cv_data()?;
                    let cv17 = self.store.read_byte(Cv::ExtendedAddressMsb as usize)?;
                    if cv17 == 192 && value == 0 {
                        return Err(PacketError::InvalidInstruction.into());
                    }
                    self.store.write_byte(Cv::ExtendedAddressLsb as usize, value)?;
                    return Ok(Some(AcknowledgeCv));
                }

                _ => true,
            },

            _ => true,
        };

        if write {
            self.store.write_byte(address as usize, packet.cv_data()?)?;
        }

        Ok(Some(AcknowledgeCv))
    }

    fn reset_to_factory_defaults(&mut self) -> Result<(), Error> {
        info!("Resetting all CVs to default");
        reset(&mut self.store)?;
        Ok(())
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
    extern crate alloc;
    use super::*;
    use crate::cv::Cv::{DecoderConfiguration, ExtendedAddressMsb, PrimaryAddress};
    use crate::handler::Op::Reset;
    use crate::testing::{MockStore, MockTimer, pkt};
    use crate::transport::packet::Error as PacketError;
    use motor::{Direction, SpeedStep};

    fn reset_packet() -> Packet {
        pkt(&[0x00, 0x00, 0x00])
    }

    fn service_verify_packet(cv_addr: u16, value: u8) -> Packet {
        let cv_addr = cv_addr - 1;
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        pkt(&[0x70 | (0b01 << 2) | addr_high, addr_low, value, 0x00])
    }

    fn service_write_packet(cv_addr: u16, value: u8) -> Packet {
        let cv_addr = cv_addr - 1;
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        pkt(&[0x70 | (0b11 << 2) | addr_high, addr_low, value, 0x00])
    }

    fn service_bit_verify_packet(cv_addr: u16, bit_pos: u8, bit_val: u8) -> Packet {
        let cv_addr = cv_addr - 1;
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        let data = 0b1110_0000 | (bit_val << 3) | bit_pos;
        pkt(&[0x70 | (0b10 << 2) | addr_high, addr_low, data, 0x00])
    }

    fn service_bit_write_packet(cv_addr: u16, bit_pos: u8, bit_val: u8) -> Packet {
        let cv_addr = cv_addr - 1;
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        let data = 0b1111_0000 | (bit_val << 3) | bit_pos;
        pkt(&[0x70 | (0b10 << 2) | addr_high, addr_low, data, 0x00])
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
            self.handler
                .store
                .read_byte(addr)
                .expect("error reading cv")
        }

        /// Send the same packet twice and return the second result.
        /// The first packet must latch (return Ok(None)).
        fn send_two_packets(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
            assert_eq!(
                self.handle(packet),
                Ok(None),
                "first service write packet should latch, not execute"
            );
            self.handle(packet)
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
        let result = harness.send_two_packets(&packet);

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
    }

    #[test]
    fn test_service_verify_byte_mismatch() {
        let store = MockStore::new().with_cv(10, 0x42);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let packet = service_verify_packet(10, 0x99);
        let result = harness.handle(&packet);

        assert_eq!(result, Ok(None));
    }

    #[test]
    fn test_service_write_byte() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_write_packet(20, 0xAB));

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
        let result = harness.send_two_packets(&packet);

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
        let store = MockStore::new().with_cv(25, 0b0000_0000);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_bit_write_packet(25, 5, 1));

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(25), 0b0010_0000);
    }

    #[test]
    fn test_service_bit_write_clear() {
        let store = MockStore::new().with_cv(25, 0xFF);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_bit_write_packet(25, 2, 0));

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(25), 0b1111_1011);
    }

    #[test]
    fn test_service_mode_timeout() {
        let mut harness = TestHarness::new();

        harness.start_timer();
        harness.set_timer_elapsed(SERVICE_MODE_TIMEOUT_MS + 1); // past timeout

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
        assert_eq!(harness.handle(&reset_packet()), Ok(Some(Op::Reset)));
        harness.set_timer_elapsed(5);

        let result = harness.send_two_packets(&service_write_packet(100, 0x55));
        assert_eq!(result, Ok(Some(Op::AcknowledgeCv)));
        assert_eq!(harness.read_cv(100), 0x55);

        let result = harness.send_two_packets(&service_verify_packet(100, 0x55));
        assert_eq!(result, Ok(Some(Op::AcknowledgeCv)));
    }

    #[test]
    fn test_bit_manipulation_all_positions() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        for bit_pos in 0..8 {
            let result = harness.send_two_packets(&service_bit_write_packet(50, bit_pos, 1));
            assert_eq!(result, Ok(Some(AcknowledgeCv)));
        }
        assert_eq!(harness.read_cv(50), 0xFF);

        for bit_pos in 0..8 {
            let result = harness.send_two_packets(&service_bit_write_packet(50, bit_pos, 0));
            assert_eq!(result, Ok(Some(AcknowledgeCv)));
        }
        assert_eq!(harness.read_cv(50), 0x00);
    }

    #[test]
    fn test_service_write_address_only() {
        let store = MockStore::new()
            .with_cv(DecoderConfiguration as u16, 0b00100000) // CV29 bit 5 set (extended addressing)
            .with_cv(19, 0x42); // CV19 consist address non-zero
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_write_packet(1, 5));

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(1), 5);
        assert_eq!(harness.read_cv(29) & 0b00100000, 0); // CV29 bit 5 cleared
        assert_eq!(harness.read_cv(19), 0); // CV19 cleared
    }

    #[test]
    fn test_service_write_address_only_invalid_range() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        // First packet latches; second executes and rejects the out-of-range value
        let invalid_err = Err(Error::Packet(PacketError::InvalidInstruction));
        assert_eq!(harness.handle(&service_write_packet(1, 0)), Ok(None));
        assert_eq!(harness.handle(&service_write_packet(1, 0)), invalid_err);

        assert_eq!(harness.handle(&service_write_packet(1, 128)), Ok(None));
        assert_eq!(harness.handle(&service_write_packet(1, 128)), invalid_err);
    }

    #[test]
    fn test_service_write_cv17_valid() {
        let store = MockStore::new().with_cv(18, 0x01); // CV18 non-zero -> address != 0
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        // CV17 = 200 is within the required 192..=231 range
        let result = harness.send_two_packets(&service_write_packet(17, 200));

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(17), 200);
    }

    #[test]
    fn test_service_write_cv17_out_of_range() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let invalid_err = Err(Error::Packet(PacketError::InvalidInstruction));

        // Below the valid 192..=231 range
        assert_eq!(
            harness.send_two_packets(&service_write_packet(17, 100)),
            invalid_err
        );
        assert_eq!(harness.read_cv(17), 0);

        // Above the valid range
        assert_eq!(
            harness.send_two_packets(&service_write_packet(17, 240)),
            invalid_err
        );
        assert_eq!(harness.read_cv(17), 0);
    }

    #[test]
    fn test_service_write_cv17_rejects_address_zero() {
        // CV18 defaults to 0; writing CV17 = 192 would form extended address 0.
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_write_packet(17, 192));

        assert_eq!(result, Err(Error::Packet(PacketError::InvalidInstruction)));
        assert_eq!(harness.read_cv(17), 0);
    }

    #[test]
    fn test_service_write_cv18_rejects_address_zero() {
        // CV17 == 192 and writing CV18 = 0 would form extended address 0.
        let store = MockStore::new().with_cv(17, 192);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_write_packet(18, 0));

        assert_eq!(result, Err(Error::Packet(PacketError::InvalidInstruction)));
        assert_eq!(harness.read_cv(18), 0);
    }

    #[test]
    fn test_service_write_cv18_valid() {
        let store = MockStore::new().with_cv(17, 200);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();

        let result = harness.send_two_packets(&service_write_packet(18, 5));

        assert_eq!(result, Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(18), 5);
    }

    #[test]
    fn test_service_write_cv31_cv32_ignored() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        // CV31/CV32 select the CV index page; this decoder does not implement indexed CVs,
        // so the write is silently ignored (no store change, no ack).
        assert_eq!(
            harness.send_two_packets(&service_write_packet(31, 0xAB)),
            Ok(None)
        );
        assert_eq!(harness.read_cv(31), 0);

        assert_eq!(
            harness.send_two_packets(&service_write_packet(32, 0xCD)),
            Ok(None)
        );
        assert_eq!(harness.read_cv(32), 0);
    }

    #[test]
    fn test_service_write_latch_first_packet_returns_none() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let result = harness.handle(&service_write_packet(20, 0xAB));

        assert_eq!(result, Ok(None));
        assert_eq!(harness.read_cv(20), 0x00); // not written
    }

    #[test]
    fn test_service_write_latch_different_second_packet_does_not_execute() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        // Latch CV 20 = 0xAB
        assert_eq!(harness.handle(&service_write_packet(20, 0xAB)), Ok(None));
        // Different data — does not match, re-latches, does not write
        assert_eq!(harness.handle(&service_write_packet(20, 0xFF)), Ok(None));
        assert_eq!(harness.read_cv(20), 0x00);
    }

    #[test]
    fn test_service_factory_defaults() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let packet = pkt(&[0b01111111, 0b00001000]);
        let result = harness.send_two_packets(&packet);

        assert_eq!(result, Ok(Some(Reboot)));
    }
}
