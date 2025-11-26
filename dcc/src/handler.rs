#[cfg(test)]
mod tests {
    use super::*;
    use heapless::Vec;
    use motor::SpeedStep;

    // Mock Timer for testing
    struct MockTimer {
        running: bool,
        elapsed: Option<usize>,
    }

    impl MockTimer {
        fn new() -> Self {
            Self {
                running: false,
                elapsed: None,
            }
        }

        fn set_elapsed(&mut self, elapsed: usize) {
            self.elapsed = Some(elapsed);
        }
    }

    impl Timer for MockTimer {
        fn start(&mut self) {
            self.running = true;
        }

        fn stop(&mut self) {
            self.running = false;
            self.elapsed = None;
        }

        fn elapsed(&self) -> Option<usize> {
            if self.running {
                self.elapsed
            } else {
                None
            }
        }
    }

    // Mock Store for testing
    struct MockStore {
        data: [u8; 1024],
        address: u16,
    }

    impl MockStore {
        fn new(address: u16) -> Self {
            Self {
                data: [0; 1024],
                address,
            }
        }

        fn with_data(address: u16, initial: &[(usize, u8)]) -> Self {
            let mut store = Self::new(address);
            for &(addr, value) in initial {
                store.data[addr] = value;
            }
            store
        }
    }

    impl Store for MockStore {
        fn read_byte(&self, address: usize) -> u8 {
            self.data[address]
        }

        fn write_byte(&mut self, address: usize, value: u8) -> Result<(), StoreError> {
            self.data[address] = value;
            Ok(())
        }

        fn address(&self) -> u16 {
            self.address
        }
    }

    // Helper functions to construct packets
    fn make_packet(bytes: &[u8]) -> Packet {
        Packet {
            data: Vec::from_slice(bytes).unwrap(),
        }
    }

    fn reset_packet() -> Packet {
        make_packet(&[0x00, 0x00, 0x00])
    }

    fn service_verify_packet(cv_addr: u16, value: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        make_packet(&[0x70 | (0b01 << 3) | addr_high, addr_low, value, 0x00])
    }

    fn service_write_packet(cv_addr: u16, value: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        make_packet(&[0x70 | (0b11 << 3) | addr_high, addr_low, value, 0x00])
    }

    fn service_bit_verify_packet(cv_addr: u16, bit_pos: u8, bit_val: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        let data = 0b1110_0000 | (bit_val << 3) | bit_pos;
        make_packet(&[0x70 | (0b10 << 3) | addr_high, addr_low, data, 0x00])
    }

    fn service_bit_write_packet(cv_addr: u16, bit_pos: u8, bit_val: u8) -> Packet {
        let addr_high = ((cv_addr >> 8) & 0x03) as u8;
        let addr_low = (cv_addr & 0xFF) as u8;
        let data = 0b1111_0000 | (bit_val << 3) | bit_pos;
        make_packet(&[0x70 | (0b10 << 3) | addr_high, addr_low, data, 0x00])
    }

    fn operation_packet(addr: u16, instruction: u8) -> Packet {
        if addr <= 127 {
            make_packet(&[addr as u8, instruction, 0x00])
        } else {
            let addr_high = 0xC0 | ((addr >> 8) as u8);
            let addr_low = (addr & 0xFF) as u8;
            make_packet(&[addr_high, addr_low, instruction, 0x00])
        }
    }

    fn speed_step_packet(addr: u16, speed_step: SpeedStep) -> Packet {
        let instruction = match speed_step {
            SpeedStep::Stop => 0b00100000,
            SpeedStep::EmergencyStop => 0b00100001,
            SpeedStep::Num(n) => 0b00100000 | n,
        };
        operation_packet(addr, instruction)
    }

    // Test harness to reduce boilerplate
    struct TestHarness {
        handler: Handler<MockTimer, MockStore>,
    }

    impl TestHarness {
        fn new(address: u16) -> Self {
            Self {
                handler: Handler::new(MockTimer::new(), MockStore::new(address)),
            }
        }

        fn with_store(store: MockStore) -> Self {
            Self {
                handler: Handler::new(MockTimer::new(), store),
            }
        }

        fn handle(&mut self, packet: Packet) -> Result<Option<Op>, Error> {
            self.handler.handle(packet)
        }

        fn set_timer_elapsed(&mut self, elapsed: usize) {
            self.handler.timer.set_elapsed(elapsed);
        }

        fn start_timer(&mut self) {
            self.handler.timer.start();
        }

        fn read_cv(&self, addr: usize) -> u8 {
            self.handler.store.read_byte(addr)
        }
    }

    #[test]
    fn test_reset_packet_starts_timer() {
        let mut harness = TestHarness::new(3);
        let result = harness.handle(reset_packet());

        assert!(matches!(result, Ok(Some(Op::Reset))));
        assert!(harness.handler.timer.running);
    }

    #[test]
    fn test_service_verify_byte_match() {
        let store = MockStore::with_data(3, &[(10, 0x42)]);
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_verify_packet(10, 0x42);
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
    }

    #[test]
    fn test_service_verify_byte_mismatch() {
        let store = MockStore::with_data(3, &[(10, 0x42)]);
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_verify_packet(10, 0x99);
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn test_service_write_byte() {
        let mut harness = TestHarness::new(3);

        harness.start_timer();
        harness.set_timer_elapsed(5);

        let packet = service_write_packet(20, 0xAB);
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
        assert_eq!(harness.read_cv(20), 0xAB);
    }

    #[test]
    fn test_service_bit_verify_match() {
        let store = MockStore::with_data(3, &[(15, 0b0000_1000)]); // bit 3 is set
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_bit_verify_packet(15, 3, 1);
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
    }

    #[test]
    fn test_service_bit_verify_mismatch() {
        let store = MockStore::with_data(3, &[(15, 0b0000_1000)]); // bit 3 is set
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_bit_verify_packet(15, 3, 0); // verify bit 3 is 0
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn test_service_bit_write_set() {
        let store = MockStore::with_data(3, &[(25, 0b0000_0000)]);
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_bit_write_packet(25, 5, 1); // set bit 5
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
        assert_eq!(harness.read_cv(25), 0b0010_0000);
    }

    #[test]
    fn test_service_bit_write_clear() {
        let store = MockStore::with_data(3, &[(25, 0xFF)]);
        let mut harness = TestHarness::with_store(store);

        harness.start_timer();
        harness.set_timer_elapsed(10);

        let packet = service_bit_write_packet(25, 2, 0); // clear bit 2
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
        assert_eq!(harness.read_cv(25), 0b1111_1011);
    }

    #[test]
    fn test_service_mode_timeout() {
        let mut harness = TestHarness::new(3);

        harness.start_timer();
        harness.set_timer_elapsed(SERVICE_MODE_TIMEOUT + 1); // past timeout

        let packet = service_verify_packet(10, 0x42);
        let result = harness.handle(packet);

        // Should not be handled as service mode packet
        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn test_operation_mode_basic_address() {
        let mut harness = TestHarness::new(3);

        let packet = speed_step_packet(3, SpeedStep::Num(50));
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::Velocity(SpeedStep::Num(50))))));
    }

    #[test]
    fn test_operation_mode_extended_address() {
        let mut harness = TestHarness::new(5000);

        let packet = speed_step_packet(5000, SpeedStep::Num(100));
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::Velocity(SpeedStep::Num(100))))));
    }

    #[test]
    fn test_operation_mode_stop() {
        let mut harness = TestHarness::new(10);

        let packet = speed_step_packet(10, SpeedStep::Stop);
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::Velocity(SpeedStep::Stop)))));
    }

    #[test]
    fn test_operation_mode_emergency_stop() {
        let mut harness = TestHarness::new(10);

        let packet = speed_step_packet(10, SpeedStep::EmergencyStop);
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(Some(Op::Velocity(SpeedStep::EmergencyStop)))));
    }

    #[test]
    fn test_packet_not_for_us() {
        let mut harness = TestHarness::new(3);

        // Send packet addressed to different decoder
        let packet = speed_step_packet(99, SpeedStep::Num(50));
        let result = harness.handle(packet);

        assert!(matches!(result, Ok(None)));
    }

    #[test]
    fn test_service_mode_sequence() {
        let mut harness = TestHarness::new(3);

        // Start with reset
        let result = harness.handle(reset_packet());
        assert!(matches!(result, Ok(Some(Op::Reset))));

        // Timer should be running, set elapsed time
        harness.set_timer_elapsed(5);

        // Write CV
        let result = harness.handle(service_write_packet(100, 0x55));
        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
        assert_eq!(harness.read_cv(100), 0x55);

        // Verify CV
        let result = harness.handle(service_verify_packet(100, 0x55));
        assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
    }

    #[test]
    fn test_bit_manipulation_all_positions() {
        let mut harness = TestHarness::new(3);
        harness.start_timer();
        harness.set_timer_elapsed(5);

        // Test writing each bit position
        for bit_pos in 0..8 {
            let packet = service_bit_write_packet(50, bit_pos, 1);
            let result = harness.handle(packet);
            assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
        }

        // All bits should be set
        assert_eq!(harness.read_cv(50), 0xFF);

        // Clear each bit
        for bit_pos in 0..8 {
            let packet = service_bit_write_packet(50, bit_pos, 0);
            let result = harness.handle(packet);
            assert!(matches!(result, Ok(Some(Op::AcknowledgeCv))));
        }

        // All bits should be clear
        assert_eq!(harness.read_cv(50), 0x00);
    }
}
