use embassy_rp::dma;
use fixed::traits::ToFixed;

use dcc::transport::{Decoder, RawDccDecoder};
use embassy_rp::Peri;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::pio_file;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, PioPin,
    ShiftDirection, StateMachine,
};

pub fn pio_decoder<'d, T: Instance, const SM: usize>(
    pio: &mut Common<'d, T>,
    sm: StateMachine<'d, T, SM>,
    dcc_input: Peri<'d, impl PioPin + 'd>,
    dma: dma::Channel<'d>,
) -> Decoder<PioDccDecoder<'d, T, SM>> {
    let prg = PioDccDecoderProgram::new(pio);
    let decoder1 = PioDccDecoder::new(pio, sm, dcc_input, dma, &prg);

    Decoder::new(decoder1)
}

/// A DCC decoder program loaded into pio instruction memory
struct PioDccDecoderProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioDccDecoderProgram<'a, PIO> {
    /// Load the program into the given pio
    fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_file!("src/transport/dcc_decoder.pio");
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// Pio backed DCC decoder.
pub struct PioDccDecoder<'d, T: Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
    dma: dma::Channel<'d>,
}

impl<'d, T: Instance, const SM: usize> PioDccDecoder<'d, T, SM> {
    /// Configure a state machine with the loaded [PioEncoderProgram]
    fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        dcc_input: Peri<'d, impl PioPin + 'd>,
        dma: dma::Channel<'d>,
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

        cfg.clock_divider = (embassy_rp::clocks::clk_sys_freq() as f64 / 400_000.0).to_fixed();

        // cfg.clock_divider = (U56F8!(125_000_000) / 400_000).to_fixed();

        cfg.shift_in.direction = ShiftDirection::Left;
        cfg.shift_in.auto_fill = true;
        cfg.shift_in.threshold = 32;

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm, dma }
    }
}

impl<'d, T: Instance, const SM: usize> RawDccDecoder for PioDccDecoder<'d, T, SM> {
    async fn read<'a>(&mut self, buff: &'a mut [u8]) -> &'a [u8] {
        assert!(buff.len() >= 8);
        let rx = self.sm.rx();

        let din: &mut [u32] = bytemuck::cast_slice_mut(buff);
        rx.dma_pull(&mut self.dma, din, true).await;

        // The PIO pads every packet to a fixed 8-byte (2-word) output and stores the
        // number of padding bytes in the final byte; the true length is `7 - padding`.
        // The count lands at buff[7] because the SM shifts it in LAST under
        // ShiftDirection::Left with autopush threshold 32, and dma_pull(bswap = true)
        // places that last-shifted byte at the end of the buffer. It is a register
        // value (not pin data), so it is NOT inverted like the data bytes below.
        // Changing the shift direction, threshold, or bswap will move this byte and
        // silently break length recovery.
        let padding = buff[7] as usize;
        if padding > 7 {
            // Corrupted/garbled packet: saturating_sub yields len 0 and the codec
            // rejects it as Undersize. Log so the drop is diagnosable instead of silent.
            debug!("dropping packet: invalid padding count {} (expected 0..=7)", padding);
        }
        let len = 7_usize.saturating_sub(padding);
        let buff = &mut buff[..len];

        // received bytes come inverted off the wire
        for byte in buff.iter_mut() {
            *byte = !*byte;
        }

        if cfg!(feature = "verbose-transport") {
            trace!("{:03} ({:08b})", buff, buff,);
        }

        buff
    }
}
