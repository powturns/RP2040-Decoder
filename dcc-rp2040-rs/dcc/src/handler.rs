use crate::cv::Cv;
use crate::cv::store::Store;
use crate::cv::store::{Error as StoreError, StoreExt, reset};
use crate::handler::Op::{AcknowledgeCv, Reboot};
use crate::transport::packet::{AdvancedOperationsInstruction, CvAccess, Error as PacketError, ManipulateBit, OperationModeInstruction, Packet, ServiceInstructionType, ServicePacket};
use crate::{FunctionGroup, Timer, is_broadcast, is_recipient};
use motor::{SpeedStep, VelocitySetpoint};

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

    /// The first of two identical service mode write packets.
    pending_service_packet: Option<Packet>,

    /// The first of two identical Programming on the Main (POM) write packets
    /// (S-9.2.1 §2.3.7.3).
    ///
    /// Cleared by any other packet addressed to this decoder, including broadcasts.
    pending_pom_packet: Option<Packet>,
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
            pending_pom_packet: None,
        }
    }

    /// Handles the packet, returning any operation that needs to be performed.
    pub fn handle(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        // must receive two pending service packets in a row with no intervening packets.
        // Calling take unconditionally ensures any valid packet drops the pending packet.
        let pending_service_packet = self.pending_service_packet.take();

        if packet.is_reset() {
            // clear any pending main track programming packet
            self.pending_pom_packet = None;

            // we may be entering service mode
            self.enter_service_mode_timer.start();
            self.handle_reset(packet).map(Some)
        } else if let Some(elapsed) = self.enter_service_mode_timer.elapsed_ms()
            && elapsed < SERVICE_MODE_TIMEOUT_MS
            && packet.service_mode_candidate()
        {
            // clear any pending main track programming packet
            self.pending_pom_packet = None;

            self.enter_service_mode_timer.start();

            self.handle_service_mode(packet, pending_service_packet)
        } else if is_recipient(packet, &self.store) {
            // this packet was specifically addressed to us (not a broadcast)
            self.enter_service_mode_timer.stop();

            self.decode_operation_command(packet)
        } else if is_broadcast(packet) {
            self.enter_service_mode_timer.stop();

            self.decode_operation_command(packet)
        } else {
            // packet not addressed to us. A pending POM write survives it.
            self.enter_service_mode_timer.stop();
            Ok(None)
        }
    }

    /// decodes the operation command.
    fn decode_operation_command(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        // Any packet addressed to us (including broadcasts) invalidates a pending POM write.
        // The matching second packet of a write consumes it below.
        let pending_pom_write = self.pending_pom_packet.take();

        match OperationModeInstruction::try_from(packet)? {
            OperationModeInstruction::CvAccess(access) => {
                if !is_broadcast(packet) {
                    self.handle_pom(packet, access, pending_pom_write)
                } else {
                    // CV access is only accepted when it is addressed to this decoder.
                    Ok(None)
                }
            }
            OperationModeInstruction::AdvancedOperations(o) => match o {
                AdvancedOperationsInstruction::SpeedStepControl(v) => Ok(Some(Op::Velocity128(v))),
            },
            OperationModeInstruction::FunctionGroup(fg) => Ok(Some(Op::SetFunctions(fg))),
            OperationModeInstruction::SpeedAndDirection(v) => {
                if matches!(v.speed_step, SpeedStep::Stop | SpeedStep::EmergencyStop) {
                    Ok(Some(Op::Velocity128(v)))
                } else {
                    // We're only supporting the Speed and direction instruction for stop / emergency
                    // stop (for broadcast support). Using it for a Speed step index would be invalid
                    // because we return a Velocity128, when it would have to be a velocity14/28.
                    Err(Error::Packet(PacketError::InvalidInstruction))
                }
            }
        }
    }

    /// Handles a Programming on the Main (POM) configuration variable access instruction.
    ///
    /// Writes are only executed when the packet is identical to `pending`, the previous
    /// write packet (S-9.2.1 §2.3.7.3).
    fn handle_pom(
        &mut self,
        packet: &Packet,
        access: CvAccess,
        pending: Option<Packet>,
    ) -> Result<Option<Op>, Error> {
        if !access.is_write() {
            // A verify is only answered over RailCom, which this decoder doesn't support.
            return Ok(None);
        }

        if pending.as_ref() != Some(packet) {
            // first of two identical packets: hold it until the second arrives
            self.pending_pom_packet = Some(packet.clone());
            return Ok(None);
        }

        let op = match access {
            CvAccess::WriteByte { cv, value } => self.write_cv(cv, value)?,
            CvAccess::ManipulateBit(mb) => self.manipulate_cv_bit(mb)?,
            CvAccess::VerifyByte { .. } => None,
        };

        // AcknowledgeCv is the service mode acknowledgement: a motor current pulse. On the main
        // it would jolt a running loco, so only a reboot is passed on.
        Ok(op.filter(|op| *op == Reboot))
    }

    fn handle_service_mode(
        &mut self,
        packet: &Packet,
        pending: Option<Packet>,
    ) -> Result<Option<Op>, Error> {
        if pending.as_ref() != Some(packet) {
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

    fn handle_reset(&mut self, _packet: &Packet) -> Result<Op, Error> {
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
        self.write_cv(packet.cv_address()?, packet.cv_data()?)
    }

    /// Writes `value` to `cv`, applying the per-CV validation and side effects.
    ///
    /// Shared by service mode and operations mode.
    fn write_cv(&mut self, cv: u16, value: u8) -> Result<Option<Op>, Error> {
        // CV31/CV32 select the CV index page (S-9.2.2). This decoder does not implement
        // indexed CVs, so the writes are silently ignored (matches core0.c:write_cv_handler).
        if cv == 31 || cv == 32 {
            return Ok(None);
        }

        let write = match Cv::try_from(cv) {
            Ok(addr) => match addr {
                Cv::PrimaryAddress => {
                    if !(1..=127).contains(&value) {
                        // return an error if the value is not in the valid primary address
                        // range
                        return Err(PacketError::InvalidInstruction.into());
                    }
                    self.store.write_byte(Cv::PrimaryAddress as usize, value)?;

                    // clear the extended address mode bit and consist addresses
                    let cv29 = self.store.read_byte(Cv::DecoderConfiguration as usize)?;
                    self.store
                        .write_byte(Cv::DecoderConfiguration as usize, cv29 & !0b00100000)?;
                    self.store.write_byte(Cv::ConsistAddress as usize, 0)?;
                    return Ok(Some(AcknowledgeCv));
                }

                // read only cvs
                Cv::ManufacturerId => {
                    // Reset all CVs to default when setting CV_8 = 8)
                    if value == 8 {
                        self.reset_to_factory_defaults()?;
                        return Ok(Some(Reboot));
                    }
                    return Ok(None);
                }

                Cv::ManufacturerVersionNumber => {
                    // ADC offset Adjustment is triggered when setting CV_7 = 7
                    if value == 7 {
                        self.store.emf_adc_offset_clear()?;
                        return Ok(Some(Reboot));
                    }

                    return Ok(None);
                }

                Cv::ExtendedAddressMsb => {
                    // CV17 holds the high 6 bits of the extended address and must be in
                    // 192..=231. CV17 == 192 with CV18 == 0 would form address 0, which is
                    // invalid. (core0.c:write_cv_handler)
                    let cv18 = self.store.read_byte(Cv::ExtendedAddressLsb as usize)?;
                    if !(192..=231).contains(&value) || (cv18 == 0 && value == 192) {
                        return Err(PacketError::InvalidInstruction.into());
                    }
                    self.store
                        .write_byte(Cv::ExtendedAddressMsb as usize, value)?;
                    return Ok(Some(AcknowledgeCv));
                }

                Cv::ExtendedAddressLsb => {
                    // CV18 holds the low 8 bits. Reject the value that would form address 0
                    // when CV17 is already at its minimum (192). (core0.c:write_cv_handler)
                    let cv17 = self.store.read_byte(Cv::ExtendedAddressMsb as usize)?;
                    if cv17 == 192 && value == 0 {
                        return Err(PacketError::InvalidInstruction.into());
                    }
                    self.store
                        .write_byte(Cv::ExtendedAddressLsb as usize, value)?;
                    return Ok(Some(AcknowledgeCv));
                }

                _ => true,
            },

            _ => true,
        };

        if write {
            self.store.write_byte(cv as usize, value)?;
        }

        Ok(Some(AcknowledgeCv))
    }

    fn reset_to_factory_defaults(&mut self) -> Result<(), Error> {
        info!("Resetting all CVs to default");
        reset(&mut self.store)?;
        Ok(())
    }

    fn manipulate_bit(&mut self, packet: &Packet) -> Result<Option<Op>, Error> {
        self.manipulate_cv_bit(
            ManipulateBit::new(
                packet.cv_address()?,
                packet.cv_data()?,
            )?
        )
    }

    /// Verifies or writes a single bit of `cv`.
    ///
    /// Shared by service mode and operations mode.
    fn manipulate_cv_bit(&mut self, mb: ManipulateBit) -> Result<Option<Op>, Error> {

        let is_write = mb.is_write();
        let d_val = mb.bit_value();
        let bit_pos = mb.bit_pos();

        let cv_addr = mb.cv_address() as usize;
        let current = self.store.read_byte(cv_addr)?;
        let mask = 1u8 << bit_pos;

        if is_write {
            // Write Bit operation - clear the current bit, and or it with the incoming bit
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
            SpeedStep::Idx(n) => n + 1, // add one to account for emergency stop being 1
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

        fn is_timer_running(&self) -> bool {
            self.handler.enter_service_mode_timer.running
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

        let packet = advanced_speed_step_packet(3, SpeedStep::Idx(50));
        let result = harness.handle(&packet);

        assert_eq!(
            result,
            Ok(Some(Op::Velocity128(VelocitySetpoint {
                speed_step: SpeedStep::Idx(50),
                direction: Direction::Reverse,
            })))
        );
    }

    #[test]
    fn test_operation_mode_extended_address() {
        let mut harness = TestHarness::with_extended_address(5000);

        let packet = advanced_speed_step_packet(5000, SpeedStep::Idx(100));
        let result = harness.handle(&packet);

        assert_eq!(
            result,
            Ok(Some(Op::Velocity128(VelocitySetpoint {
                speed_step: SpeedStep::Idx(100),
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

    // Broadcast handling (S-9.2 §B, S-9.2.1 §2.2): address 0 is the broadcast address, and the
    // Broadcast Stop packet is required for decoders.

    /// Asserts the result is a velocity op with the given speed step. Direction is not checked:
    /// S-9.2 and S-9.2.1 allow decoders to ignore the direction in a broadcast stop.
    fn assert_velocity_step(result: Result<Option<Op>, Error>, expected: SpeedStep) {
        match result {
            Ok(Some(Op::Velocity128(v))) => assert_eq!(v.speed_step, expected),
            other => panic!("expected Velocity128 with {expected:?}, got {other:?}"),
        }
    }

    #[test]
    fn test_broadcast_128_step_stop() {
        let mut harness = TestHarness::with_address(3);

        let result = harness.handle(&advanced_speed_step_packet(0, SpeedStep::Stop));

        assert_velocity_step(result, SpeedStep::Stop);
    }

    #[test]
    fn test_broadcast_128_step_emergency_stop() {
        let mut harness = TestHarness::with_address(3);

        let result = harness.handle(&advanced_speed_step_packet(0, SpeedStep::EmergencyStop));

        assert_velocity_step(result, SpeedStep::EmergencyStop);
    }

    #[test]
    fn test_broadcast_stop_packet() {
        // S-9.2 Broadcast Stop: 00000000 01DC000S, S = 0 -> bring the locomotive to a stop.
        // Must work for every value of the D and C bits.
        for dc in 0..=0b11u8 {
            let mut harness = TestHarness::with_address(3);
            let instruction = 0b0100_0000 | (dc << 4);

            let result = harness.handle(&operation_packet(0, &[instruction]));

            assert_velocity_step(result, SpeedStep::Stop);
        }
    }

    #[test]
    fn test_broadcast_speed_step() {
        // ensure we don't respond to any of the other broadcast speed steps for now.
        // 01DCSSSS: the 28 speed steps are CSSSS = x0010..=x1111 (S-9.2 §B), for both
        // directions (D). SSSS = 0000 / 0001 are stop / emergency stop and are tested separately.
        for d in 0..=1u8 {
            for c in 0..=1u8 {
                for speed in 0b0010..=0b1111u8 {
                    let mut harness = TestHarness::with_address(3);
                    let instruction = 0b0100_0000 | (d << 5) | (c << 4) | speed;

                    let result = harness.handle(&operation_packet(0, &[instruction]));

                    assert_eq!(
                        result,
                        Err(Error::Packet(PacketError::InvalidInstruction)),
                        "instruction {instruction:#010b}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_broadcast_emergency_stop_packet() {
        // S-9.2 Broadcast Stop: 00000000 01DC000S, S = 1 -> stop delivering energy to the motor.
        for dc in 0..=0b11u8 {
            let mut harness = TestHarness::with_address(3);
            let instruction = 0b0100_0001 | (dc << 4);

            let result = harness.handle(&operation_packet(0, &[instruction]));

            assert_velocity_step(result, SpeedStep::EmergencyStop);
        }
    }

    #[test]
    fn test_broadcast_stop_with_extended_address() {
        // Broadcast applies regardless of the decoder's addressing mode.
        let mut harness = TestHarness::with_extended_address(5000);

        let result = harness.handle(&advanced_speed_step_packet(0, SpeedStep::EmergencyStop));

        assert_velocity_step(result, SpeedStep::EmergencyStop);
    }

    #[test]
    fn test_broadcast_stop_with_service_mode_timer_running() {
        // A broadcast stop shortly after a reset is not a service mode candidate (address 0 is
        // outside 112..=127) and must still be executed.
        let mut harness = TestHarness::with_address(3);
        harness.enter_service_mode();

        let result = harness.handle(&advanced_speed_step_packet(0, SpeedStep::EmergencyStop));

        assert_velocity_step(result, SpeedStep::EmergencyStop);
        assert!(!harness.is_timer_running())
    }

    #[test]
    fn test_broadcast_reset_is_still_reset() {
        // 00000000 00000000 is the broadcast Digital Decoder Reset, not a speed instruction.
        let mut harness = TestHarness::with_address(3);

        assert_eq!(
            harness.handle(&operation_packet(0, &[0x00])),
            Ok(Some(Reset))
        );
    }

    #[test]
    fn test_packet_not_for_us() {
        let mut harness = TestHarness::with_address(3);

        // Send packet addressed to different decoder
        let packet = advanced_speed_step_packet(99, SpeedStep::Idx(50));
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

    // S-9.2.3 §E: a decoder must not execute a Verify or Write unless it has received two
    // identical service mode packets without any intervening valid packets.

    #[test]
    fn test_service_write_intervening_reset_does_not_execute() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let write = service_write_packet(20, 0xAB);
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.handle(&reset_packet()), Ok(Some(Reset)));

        // the reset dropped the latched packet, so this write latches again
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.read_cv(20), 0x00);

        // a following identical packet with nothing in between executes
        assert_eq!(harness.handle(&write), Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(20), 0xAB);
    }

    #[test]
    fn test_service_verify_intervening_reset_does_not_execute() {
        let mut harness = TestHarness::with_store(MockStore::new().with_cv(20, 0xAB));
        harness.enter_service_mode();

        let verify = service_verify_packet(20, 0xAB);
        assert_eq!(harness.handle(&verify), Ok(None));
        assert_eq!(harness.handle(&reset_packet()), Ok(Some(Reset)));
        assert_eq!(harness.handle(&verify), Ok(None));
    }

    #[test]
    fn test_service_latch_not_carried_across_sessions() {
        // A 5-packet verify sequence executes on packets 2 and 4 and latches packet 5. That latch
        // must not let the first identical packet after the next reset execute on its own.
        let mut harness = TestHarness::with_store(MockStore::new().with_cv(20, 0xAB));
        harness.enter_service_mode();

        let verify = service_verify_packet(20, 0xAB);
        let expected = [None, Some(AcknowledgeCv), None, Some(AcknowledgeCv), None];
        for (i, expected) in expected.into_iter().enumerate() {
            assert_eq!(
                harness.handle(&verify),
                Ok(expected),
                "verify packet {}",
                i + 1
            );
        }

        // new session
        assert_eq!(harness.handle(&reset_packet()), Ok(Some(Reset)));
        assert_eq!(
            harness.handle(&verify),
            Ok(None),
            "first packet must only latch"
        );
        assert_eq!(harness.handle(&verify), Ok(Some(AcknowledgeCv)));
    }

    #[test]
    fn test_service_idle_packet_exits_service_mode() {
        // S-9.2.3 §C.1: any valid non service mode packet exits service mode, including idle.
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let write = service_write_packet(20, 0xAB);
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.handle(&pkt(&[0xFF, 0x00])), Ok(None));
        assert!(
            !harness.is_timer_running(),
            "idle packet should exit service mode"
        );

        // no longer in service mode, so the writes are not executed
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.read_cv(20), 0x00);
    }

    #[test]
    fn test_service_packet_for_other_decoder_exits_service_mode() {
        let mut harness = TestHarness::with_address(3);
        harness.enter_service_mode();

        let write = service_write_packet(20, 0xAB);
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(
            harness.handle(&advanced_speed_step_packet(99, SpeedStep::Idx(10))),
            Ok(None)
        );
        assert!(
            !harness.is_timer_running(),
            "non service packet should exit service mode"
        );

        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.read_cv(20), 0x00);
    }

    #[test]
    fn test_service_latch_dropped_after_timeout() {
        // The latched packet must not survive leaving service mode via the 20ms timeout.
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let write = service_write_packet(20, 0xAB);
        assert_eq!(harness.handle(&write), Ok(None));

        // service mode times out; this packet is ignored and drops the latch
        harness.set_timer_elapsed(SERVICE_MODE_TIMEOUT_MS + 1);
        assert_eq!(harness.handle(&write), Ok(None));

        // re-enter service mode: the first write only latches
        assert_eq!(harness.handle(&reset_packet()), Ok(Some(Reset)));
        harness.set_timer_elapsed(5);
        assert_eq!(harness.handle(&write), Ok(None));
        assert_eq!(harness.read_cv(20), 0x00);
        assert_eq!(harness.handle(&write), Ok(Some(AcknowledgeCv)));
        assert_eq!(harness.read_cv(20), 0xAB);
    }

    #[test]
    fn test_service_factory_defaults() {
        let mut harness = TestHarness::new();
        harness.enter_service_mode();

        let packet = pkt(&[0b01111111, 0b00001000]);
        let result = harness.send_two_packets(&packet);

        assert_eq!(result, Ok(Some(Reboot)));
    }

    // CV7 (Manufacturer Version Number) is read only. Writing CV7 = 7 triggers an EMF ADC offset
    // re-calibration (clears CV172 and reboots). Any other value is ignored (core0.c:285-292).
    const CV7_VERSION: u8 = 1;
    const EMF_ADC_OFFSET: u8 = 0x20;

    fn cv7_harness() -> TestHarness {
        let store = MockStore::new()
            .with_cv(Cv::ManufacturerVersionNumber as u16, CV7_VERSION)
            .with_cv(Cv::EmfAdcOffset as u16, EMF_ADC_OFFSET);
        let mut harness = TestHarness::with_store(store);
        harness.enter_service_mode();
        harness
    }

    #[test]
    fn test_service_write_cv7_7_clears_adc_offset_and_reboots() {
        let mut harness = cv7_harness();

        let result = harness.send_two_packets(&service_write_packet(7, 7));

        assert_eq!(result, Ok(Some(Reboot)));
        assert_eq!(
            harness.read_cv(Cv::EmfAdcOffset as usize),
            u8::MAX,
            "EMF ADC offset should be cleared"
        );
        assert_eq!(
            harness.read_cv(Cv::ManufacturerVersionNumber as usize),
            CV7_VERSION,
            "CV7 is read only"
        );
    }

    #[test]
    fn test_service_write_cv7_other_values_ignored() {
        for value in (0..=u8::MAX).filter(|&v| v != 7) {
            let mut harness = cv7_harness();

            let result = harness.send_two_packets(&service_write_packet(7, value));

            assert_eq!(result, Ok(None), "CV7 = {value} should not reboot or ack");
            assert_eq!(
                harness.read_cv(Cv::EmfAdcOffset as usize),
                EMF_ADC_OFFSET,
                "CV7 = {value} should not clear the EMF ADC offset"
            );
            assert_eq!(
                harness.read_cv(Cv::ManufacturerVersionNumber as usize),
                CV7_VERSION,
                "CV7 = {value}: CV7 is read only"
            );
        }
    }

    // Programming on the Main (POM)

    const POM_ADDR: u8 = 3;

    /// Builds a long form CV access packet: `gg` = 0b01 verify, 0b11 write byte,
    /// 0b10 bit manipulation.
    fn pom_packet(addr: u16, gg: u8, cv: u16, data: u8) -> Packet {
        let a = cv - 1;
        operation_packet(
            addr,
            &[0b1110_0000 | (gg << 2) | (a >> 8) as u8, a as u8, data],
        )
    }

    fn pom_write_packet(cv: u16, value: u8) -> Packet {
        pom_packet(POM_ADDR as u16, 0b11, cv, value)
    }

    fn pom_bit_packet(cv: u16, write: bool, bit_pos: u8, bit_val: u8) -> Packet {
        let packed = 0b1110_0000 | ((write as u8) << 4) | (bit_val << 3) | bit_pos;
        pom_packet(POM_ADDR as u16, 0b10, cv, packed)
    }

    fn pom_harness() -> TestHarness {
        TestHarness::with_address(POM_ADDR)
    }

    #[test]
    fn test_pom_write_byte_requires_two_identical_packets() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        assert_eq!(harness.handle(&packet), Ok(None));
        assert_eq!(harness.read_cv(3), 0, "first packet must not write");

        assert_eq!(harness.handle(&packet), Ok(None), "POM never acknowledges");
        assert_eq!(harness.read_cv(3), 0x42);
    }

    #[test]
    fn test_pom_write_byte_with_extended_address() {
        let mut harness = TestHarness::with_extended_address(1000);
        let packet = pom_packet(1000, 0b11, 3, 0x42);

        harness.handle(&packet).unwrap();
        harness.handle(&packet).unwrap();

        assert_eq!(harness.read_cv(3), 0x42);
    }

    #[test]
    fn test_pom_write_repeated_packets_are_harmless() {
        // command stations repeat POM packets: arm, execute, arm, execute
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        for _ in 0..4 {
            assert_eq!(harness.handle(&packet), Ok(None));
        }

        assert_eq!(harness.read_cv(3), 0x42);
    }

    #[test]
    fn test_pom_write_invalidated_by_packet_to_us() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        harness.handle(&packet).unwrap();
        harness
            .handle(&advanced_speed_step_packet(
                POM_ADDR as u16,
                SpeedStep::Idx(10),
            ))
            .unwrap();
        harness.handle(&packet).unwrap();

        assert_eq!(
            harness.read_cv(3),
            0,
            "intervening packet must cancel the write"
        );

        // the write after the cancelled one acts as a new first packet
        harness.handle(&packet).unwrap();
        assert_eq!(harness.read_cv(3), 0x42);
    }

    #[test]
    fn test_pom_write_invalidated_by_invalid_packet_to_us() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        harness.handle(&packet).unwrap();
        // GG = 00 is reserved: an error, but still a packet to this decoder
        assert!(
            harness
                .handle(&pom_packet(POM_ADDR as u16, 0b00, 3, 0x42))
                .is_err()
        );
        harness.handle(&packet).unwrap();

        assert_eq!(harness.read_cv(3), 0);
    }

    #[test]
    fn test_pom_write_invalidated_by_broadcast() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        harness.handle(&packet).unwrap();
        harness
            .handle(&advanced_speed_step_packet(0, SpeedStep::Stop))
            .unwrap();
        harness.handle(&packet).unwrap();

        assert_eq!(harness.read_cv(3), 0);
    }

    #[test]
    fn test_pom_write_invalidated_by_reset_packet() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        harness.handle(&packet).unwrap();
        harness.handle(&reset_packet()).unwrap();
        harness.handle(&packet).unwrap();

        assert_eq!(harness.read_cv(3), 0);
    }

    #[test]
    fn test_pom_write_survives_packet_to_other_decoder() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        harness.handle(&packet).unwrap();
        harness
            .handle(&advanced_speed_step_packet(
                POM_ADDR as u16 + 1,
                SpeedStep::Idx(10),
            ))
            .unwrap();
        harness.handle(&packet).unwrap();

        assert_eq!(harness.read_cv(3), 0x42);
    }

    #[test]
    fn test_pom_write_different_packets_replace_pending() {
        let mut harness = pom_harness();

        harness.handle(&pom_write_packet(3, 0x11)).unwrap();
        harness.handle(&pom_write_packet(3, 0x22)).unwrap();
        assert_eq!(harness.read_cv(3), 0, "neither write is complete");

        harness.handle(&pom_write_packet(3, 0x22)).unwrap();
        assert_eq!(harness.read_cv(3), 0x22);
    }

    #[test]
    fn test_pom_write_bit() {
        let mut harness = TestHarness::with_store(
            MockStore::new()
                .with_cv(PrimaryAddress as u16, POM_ADDR)
                .with_cv(3, 0b0000_0001),
        );

        let set = pom_bit_packet(3, true, 7, 1);
        harness.handle(&set).unwrap();
        assert_eq!(
            harness.read_cv(3),
            0b0000_0001,
            "first packet must not write"
        );
        assert_eq!(harness.handle(&set), Ok(None));
        assert_eq!(harness.read_cv(3), 0b1000_0001);

        let clear = pom_bit_packet(3, true, 0, 0);
        harness.handle(&clear).unwrap();
        harness.handle(&clear).unwrap();
        assert_eq!(harness.read_cv(3), 0b1000_0000);
    }

    #[test]
    fn test_pom_verify_is_a_noop() {
        let mut harness = TestHarness::with_store(
            MockStore::new()
                .with_cv(PrimaryAddress as u16, POM_ADDR)
                .with_cv(3, 0x42),
        );

        // matching verify byte and verify bit: no RailCom, so no response at all
        let verify_byte = pom_packet(POM_ADDR as u16, 0b01, 3, 0x42);
        let verify_bit = pom_bit_packet(3, false, 1, 1);
        for packet in [&verify_byte, &verify_byte, &verify_bit, &verify_bit] {
            assert_eq!(harness.handle(packet), Ok(None));
        }
        assert_eq!(harness.read_cv(3), 0x42);
    }

    #[test]
    fn test_pom_verify_invalidates_pending_write() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(3, 0x42);

        harness.handle(&packet).unwrap();
        harness
            .handle(&pom_packet(POM_ADDR as u16, 0b01, 3, 0))
            .unwrap();
        harness.handle(&packet).unwrap();

        assert_eq!(harness.read_cv(3), 0);
    }

    #[test]
    fn test_pom_write_to_broadcast_address_is_ignored() {
        let mut harness = pom_harness();
        let packet = pom_packet(0, 0b11, 3, 0x42);

        assert_eq!(harness.handle(&packet), Ok(None));
        assert_eq!(harness.handle(&packet), Ok(None));
        assert_eq!(harness.read_cv(3), 0);
    }

    #[test]
    fn test_pom_write_shares_service_mode_validation() {
        // CV1 out of range is rejected
        let mut harness = pom_harness();
        let packet = pom_write_packet(PrimaryAddress as u16, 0);
        harness.handle(&packet).unwrap();
        assert_eq!(
            harness.handle(&packet),
            Err(Error::Packet(PacketError::InvalidInstruction))
        );
        assert_eq!(harness.read_cv(PrimaryAddress as usize), POM_ADDR);

        // CV31 / CV32 are ignored
        let mut harness = pom_harness();
        let packet = pom_write_packet(31, 5);
        harness.handle(&packet).unwrap();
        assert_eq!(harness.handle(&packet), Ok(None));
        assert_eq!(harness.read_cv(31), 0);
    }

    #[test]
    fn test_pom_write_primary_address_changes_address_without_ack() {
        let mut harness = pom_harness();
        let packet = pom_write_packet(PrimaryAddress as u16, 5);

        harness.handle(&packet).unwrap();
        assert_eq!(harness.handle(&packet), Ok(None));
        assert_eq!(harness.read_cv(PrimaryAddress as usize), 5);

        // the decoder now answers to the new address only
        assert_eq!(harness.handle(&packet), Ok(None), "old address is ignored");
        let v = advanced_speed_step_packet(5, SpeedStep::Idx(10));
        assert!(matches!(harness.handle(&v), Ok(Some(Op::Velocity128(_)))));
    }

    #[test]
    fn test_pom_write_cv8_8_resets_and_reboots() {
        let mut harness = TestHarness::with_store(
            MockStore::new()
                .with_cv(PrimaryAddress as u16, POM_ADDR)
                .with_cv(3, 0x42),
        );
        let packet = pom_write_packet(Cv::ManufacturerId as u16, 8);

        harness.handle(&packet).unwrap();
        assert_eq!(harness.handle(&packet), Ok(Some(Reboot)));
        assert_ne!(harness.read_cv(3), 0x42, "CVs reset to defaults");
    }

    #[test]
    fn test_pom_write_cv7_7_clears_adc_offset_and_reboots() {
        let mut harness = TestHarness::with_store(
            MockStore::new()
                .with_cv(PrimaryAddress as u16, POM_ADDR)
                .with_cv(Cv::ManufacturerVersionNumber as u16, CV7_VERSION)
                .with_cv(Cv::EmfAdcOffset as u16, EMF_ADC_OFFSET),
        );
        let packet = pom_write_packet(Cv::ManufacturerVersionNumber as u16, 7);

        harness.handle(&packet).unwrap();
        assert_eq!(harness.handle(&packet), Ok(Some(Reboot)));
        assert_eq!(harness.read_cv(Cv::EmfAdcOffset as usize), u8::MAX);
    }
}
