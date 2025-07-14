use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;

use embassy_rp::gpio::Pull;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, PioPin, ShiftDirection, StateMachine,
};
use embassy_rp::pio::program::pio_file;

/// A DCC decoder program loaded into pio instruction memory
pub struct PioDccDecoderProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioDccDecoderProgram<'a, PIO> {
    /// Load the program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_file!("src/pio/decoder/dcc_decoder.pio");
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// Pio backed DCC decoder.
pub struct PioDccDecoder<'d, T:Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
}

impl<'d, T: Instance, const SM: usize> PioDccDecoder<'d, T, SM> {
    /// Configure a state machine with the loaded [PioEncoderProgram]
    pub fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        dcc_input: impl PioPin,
        program: &PioDccDecoderProgram<'d, T>,
    ) -> Self {
        let mut dcc_input = pio.make_pio_pin(dcc_input);

        dcc_input.set_pull(Pull::Up);
        sm.set_pin_dirs(PioDirection::In, &[&dcc_input]);

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);
        cfg.set_in_pins(&[&dcc_input]);
        cfg.set_jmp_pin(&dcc_input);
        cfg.fifo_join = FifoJoin::RxOnly;

        // use a clock divider that produces 2.5us per instruction:
        // The main clock runs at 125MHZ, we want to figure out what to scale that
        // by to make each cycle take 2.5us.
        // 1_000_000 us in a second / 2.5 us = 400_000
        cfg.clock_divider = (U56F8!(125_000_000) / 400_000).to_fixed();

        cfg.shift_in.direction = ShiftDirection::Left;
        cfg.shift_in.auto_fill = true;
        cfg.shift_in.threshold = 32;

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm }
    }

    pub async fn read(&mut self) -> u32 {
        // tested
        self.sm.rx().wait_pull().await

        // untested

        // orignal-ish:
        // loop {
        //     let (rx, tx) = self.sm.rx_tx();

        // todo!()
        // rx.dma_pull()
        // }
    }
}