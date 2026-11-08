use fixed::traits::ToFixed;

use dcc::transport::{Decoder, RawDccDecoder};
use dcc_pio_frame::{Frame, FrameAssembler};
use embassy_rp::Peri;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::pio_file;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, Irq, LoadedProgram, PioPin,
    ShiftDirection, StateMachine, StatusSource,
};

/// RX FIFO level at which the PIO wakes the CPU in the middle of a long frame.
///
/// Words are at least 27 bit times (>= 3.1ms) apart, so the 8 entry FIFO leaves the CPU at
/// least 4 words (>= 12.5ms) to respond. Frames of up to this many words (12 bytes) wake the
/// CPU only once, when they end.
const MID_FRAME_WAKE_LEVEL: u8 = 4;

pub fn pio_decoder<'d, T: Instance, const SM: usize>(
    pio: &mut Common<'d, T>,
    sm: StateMachine<'d, T, SM>,
    irq: Irq<'d, T, 0>,
    dcc_input: Peri<'d, impl PioPin + 'd>,
) -> Decoder<PioDccDecoder<'d, T, SM>> {
    let prg = PioDccDecoderProgram::new(pio);
    let decoder1 = PioDccDecoder::new(pio, sm, irq, dcc_input, &prg);

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
    /// Raised by the program when a frame ends (or the FIFO fills mid-frame).
    irq: Irq<'d, T, 0>,
    /// Kept across reads so a FIFO overflow is handled even when it straddles two frames.
    assembler: FrameAssembler,
}

impl<'d, T: Instance, const SM: usize> PioDccDecoder<'d, T, SM> {
    /// Configure a state machine with the loaded [PioEncoderProgram]
    fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        irq: Irq<'d, T, 0>,
        dcc_input: Peri<'d, impl PioPin + 'd>,
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

        // The program pushes manually; each word carries a sentinel bit, so the ISR is never
        // filled to 32 bits and autopush stays off. See dcc::transport::frame for the format.
        cfg.shift_in.direction = ShiftDirection::Left;
        cfg.shift_in.auto_fill = false;

        // `mov x, status` reads all-ones while the RX FIFO holds fewer than this many words
        cfg.status_sel = StatusSource::RxFifoLevel;
        cfg.status_n = MID_FRAME_WAKE_LEVEL;

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            sm,
            irq,
            assembler: FrameAssembler::new(),
        }
    }
}

impl<'d, T: Instance, const SM: usize> RawDccDecoder for PioDccDecoder<'d, T, SM> {
    async fn read<'a>(&mut self, buff: &'a mut [u8]) -> &'a [u8] {
        loop {
            // Drain before sleeping: words of the next frame may already be queued behind the one
            // returned last time, and their IRQ may have been consumed with it. The flag is only a
            // hint; the words themselves mark where frames end.
            loop {
                // A push to a full FIFO (the CPU was blocked, e.g. by a flash erase) is dropped and
                // sets the stall flag. Checked before every pull, so nothing has been pulled since
                // the drop: the words queued now all precede the gap. The level is read first so
                // that a drop between the two reads can only undercount, discarding more.
                let queued = self.sm.rx().level() as usize;
                if self.sm.rx().stalled() {
                    debug!(
                        "RX FIFO overflow: discarding the frame after {} queued words",
                        queued
                    );
                    self.assembler.gap_after(queued);
                }

                let Some(word) = self.sm.rx().try_pull() else {
                    break;
                };

                match self.assembler.push(word, buff) {
                    Frame::Partial => {}
                    Frame::Complete(len) => {
                        let buff = &buff[..len];

                        if cfg!(feature = "verbose-transport") {
                            trace!("{:03} ({:08b})", buff, buff);
                        }

                        return buff;
                    }
                    Frame::Oversize(len) => debug!("dropping frame: length {} too long", len),
                    Frame::Malformed(word) => {
                        debug!("dropping frame: malformed word {:#010x}", word)
                    }
                    Frame::Overrun => debug!("dropping frame: lost a word to RX FIFO overflow"),
                }
            }

            self.irq.wait().await;
        }
    }
}
