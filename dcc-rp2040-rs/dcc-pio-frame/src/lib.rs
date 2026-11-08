//! Reassembly of DCC frames from the word stream of the RP2040 PIO decoder program
//! (`application/src/transport/dcc_decoder.pio`).
//!
//! Everything here is specific to that program's output format; keep the two in step. A
//! different decoder implementation would not produce these words.
//!
//! The program emits one 32-bit word per (up to) three received bytes. Each byte is a 9-bit token
//! `[d7..d0 s]`:
//! - the data bits are inverted: the program samples the line 75us after each rising edge, when
//!   a DCC `1` (short half-bit) is already low and a DCC `0` (long half-bit) is still high;
//! - `s` records the bit that followed the byte: `1` = a data byte start bit (another byte
//!   follows), `0` = the packet end bit.
//!
//! Tokens are preceded by a sentinel `1`, so a word holding `n` tokens has its highest set bit at
//! `9 * n`:
//!
//! ```text
//!   0..0 1 [d7..d0 s] [d7..d0 s] [d7..d0 s]
//!          [ byte 1 ] [ byte 2 ] [ byte 3 ]
//! ```
//!
//! A frame always ends at a word boundary, because the program flushes the word when it sees the
//! packet end bit.
//!
//! If the RX FIFO is full when the program pushes, that word is lost. The words around the gap
//! must not be joined into one frame: [FrameAssembler::gap_after] marks where the gap is, and the
//! assembler then discards everything up to and including the next packet end bit.
#![no_std]

// Length of a single token in bits (8 data bits + separator, where the separator is a data byte
// start bit or packet end bit)
const TOKEN_BITS: u32 = 9;

/// The result of feeding one word to a [FrameAssembler].
#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Frame {
    /// The frame continues in the next word.
    Partial,

    /// A frame of the given length is complete.
    Complete(usize),

    /// A frame of the given length ended but did not fit in the buffer; it was discarded.
    Oversize(usize),

    /// The word did not carry a valid sentinel; any partial frame was discarded.
    Malformed(u32),

    /// A frame that lost a word to an RX FIFO overflow ended; it was discarded.
    Overrun,
}

/// Accumulates PIO words into frames.
#[derive(Default)]
pub struct FrameAssembler {
    len: usize,
    /// Intact words still to be decoded before the gap left by an RX FIFO overflow.
    words_before_gap: Option<usize>,
    /// Discarding words until the next packet end bit, because of a gap.
    resync: bool,
}

impl FrameAssembler {
    pub const fn new() -> Self {
        Self {
            len: 0,
            words_before_gap: None,
            resync: false,
        }
    }

    /// Records that a word was lost after the `queued` words currently waiting in the FIFO.
    ///
    /// Those words are intact and are decoded normally. The frame in progress after them is
    /// discarded, along with every word up to and including the next packet end bit.
    pub fn gap_after(&mut self, queued: usize) {
        self.words_before_gap = Some(queued);
    }

    /// Unpacks `word` into `buf`, starting where the previous word left off.
    ///
    /// The same `buf` must be passed for every word of a frame. Bytes beyond `buf.len()` are
    /// counted but not stored.
    pub fn push(&mut self, word: u32, buf: &mut [u8]) -> Frame {
        match self.words_before_gap {
            Some(0) => {
                self.words_before_gap = None;
                self.resync = true;
            }
            Some(n) => self.words_before_gap = Some(n - 1),
            None => {}
        }

        let sentinel_position = match word.checked_ilog2() {
            // check that the position is non-zero and sits just above a
            // whole number of tokens: bit 9, 18 or 27
            Some(bit) if bit != 0 && bit % TOKEN_BITS == 0 => bit,
            _ => {
                self.len = 0;
                return Frame::Malformed(word);
            }
        };

        // at most 3 tokens: a u32 cannot hold a sentinel above bit 27
        // notice we count in reverse! the highest token is the first byte received.
        for i in (0..sentinel_position / TOKEN_BITS).rev() {
            let token = word >> (i * TOKEN_BITS);

            if let Some(slot) = buf.get_mut(self.len) {
                // Shift the token one right to drop the data start bit / packet end bit
                // data bits are inverted when passed from pio driver, flip them to get the true value
                // the u8 cast, takes the lower 8 bits
                *slot = !(token >> 1) as u8;
            }
            self.len += 1;

            // if the `s` bit of the token is 0, it's a packet end bit, otherwise there's another
            // data byte.
            if token & 0b000000001 == 0 {
                // packet end bit
                let len = core::mem::take(&mut self.len);
                return if core::mem::take(&mut self.resync) {
                    Frame::Overrun
                } else if len <= buf.len() {
                    Frame::Complete(len)
                } else {
                    Frame::Oversize(len)
                };
            }
        }

        Frame::Partial
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    /// S-9.2.1.1 maximum frame length, as used by the firmware's frame buffer.
    const MAX_FRAME_LEN: usize = 32;

    /// Encodes bytes the way the PIO program does.
    fn encode(frame: &[u8]) -> Vec<u32> {
        let mut words = Vec::new();
        for (chunk_idx, chunk) in frame.chunks(3).enumerate() {
            let mut word = 1u32;
            for (i, &byte) in chunk.iter().enumerate() {
                let last = chunk_idx * 3 + i == frame.len() - 1;
                word = (word << 9) | ((!byte as u32 & 0xFF) << 1) | (!last as u32);
            }
            words.push(word);
        }
        words
    }

    fn decode(words: &[u32], buf: &mut [u8]) -> Vec<Frame> {
        let mut asm = FrameAssembler::new();
        words.iter().map(|&w| asm.push(w, buf)).collect()
    }

    #[test]
    fn single_word_frame() {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let frame = [0x03, 0x3F, 0x3C];
        assert_eq!(decode(&encode(&frame), &mut buf), [Frame::Complete(3)]);
        assert_eq!(&buf[..3], &frame);
    }

    #[test]
    fn words_captured_from_pio_simulation() {
        let mut buf = [0u8; MAX_FRAME_LEN];
        assert_eq!(encode(&[0x03, 0x3F, 0x3C]), [0x0FE7_0386]);
        assert_eq!(decode(&[0x0FE7_0386], &mut buf), [Frame::Complete(3)]);
        assert_eq!(&buf[..3], &[0x03, 0x3F, 0x3C]);

        let words = [0x09E7_4F81, 0x0B0F_B746];
        assert_eq!(encode(&[0xC3, 0x2C, 0x3F, 0x9E, 0x12, 0x5C]), words);
        assert_eq!(
            decode(&words, &mut buf),
            [Frame::Partial, Frame::Complete(6)]
        );
        assert_eq!(&buf[..6], &[0xC3, 0x2C, 0x3F, 0x9E, 0x12, 0x5C]);
    }

    #[test]
    fn multi_word_frames() {
        for len in 1..=MAX_FRAME_LEN {
            let frame: Vec<u8> = (0..len as u8).map(|b| b.wrapping_mul(37) ^ 0xA5).collect();
            let words = encode(&frame);
            let mut buf = [0u8; MAX_FRAME_LEN];
            let out = decode(&words, &mut buf);
            assert_eq!(out.last(), Some(&Frame::Complete(len)), "len {len}");
            assert!(out[..out.len() - 1].iter().all(|f| *f == Frame::Partial));
            assert_eq!(&buf[..len], frame.as_slice());
        }
    }

    #[test]
    fn back_to_back_frames() {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let mut asm = FrameAssembler::new();
        for frame in [
            &[0xC3, 0x2C, 0x3F, 0x9E, 0x12, 0x5C][..],
            &[0xFF, 0x00, 0xFF][..],
        ] {
            let mut last = Frame::Partial;
            for w in encode(frame) {
                last = asm.push(w, &mut buf);
            }
            assert_eq!(last, Frame::Complete(frame.len()));
            assert_eq!(&buf[..frame.len()], frame);
        }
    }

    #[test]
    fn oversize_frame_is_dropped_and_next_frame_decodes() {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let mut asm = FrameAssembler::new();
        let big = [0x55u8; 40];
        let mut last = Frame::Partial;
        for w in encode(&big) {
            last = asm.push(w, &mut buf);
        }
        assert_eq!(last, Frame::Oversize(40));

        let small = [0x03, 0x3F, 0x3C];
        assert_eq!(asm.push(encode(&small)[0], &mut buf), Frame::Complete(3));
        assert_eq!(&buf[..3], &small);
    }

    /// Feeds `words` and returns the frames that completed, as byte vectors.
    fn frames(asm: &mut FrameAssembler, words: &[u32]) -> Vec<Result<Vec<u8>, Frame>> {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let mut out = Vec::new();
        for &w in words {
            match asm.push(w, &mut buf) {
                Frame::Partial => {}
                Frame::Complete(len) => out.push(Ok(buf[..len].to_vec())),
                other => out.push(Err(other)),
            }
        }
        out
    }

    #[test]
    fn gap_mid_frame_discards_only_the_broken_frame() {
        let a = [0x03, 0x3F, 0x3C];
        let b = [0xC3, 0x2C, 0x3F, 0x9E, 0x12, 0x5C];
        let c = [0x04, 0x3F, 0x3B];
        let wb = encode(&b);
        let mut asm = FrameAssembler::new();

        // `a` and the first word of `b` were queued; the second word of `b` was lost
        asm.gap_after(2);
        let mut words = encode(&a);
        words.push(wb[0]);
        words.extend(encode(&c));
        assert_eq!(
            frames(&mut asm, &words),
            [Ok(a.to_vec()), Err(Frame::Overrun)],
            "c's single word must not be joined onto b's first word"
        );

        // decoding continues normally afterwards
        assert_eq!(frames(&mut asm, &encode(&a)), [Ok(a.to_vec())]);
    }

    #[test]
    fn gap_at_frame_start_does_not_accept_the_truncated_frame() {
        // 6-byte frame whose last 3 bytes happen to XOR to 0 on their own
        let b = [0x10, 0x20, 0x30, 0x01, 0x02, 0x03];
        let wb = encode(&b);
        let a = [0x03, 0x3F, 0x3C];
        let mut asm = FrameAssembler::new();

        // `a` was queued; `b`'s first word was lost, its second word arrived
        asm.gap_after(1);
        let mut words = encode(&a);
        words.push(wb[1]);
        assert_eq!(
            frames(&mut asm, &words),
            [Ok(a.to_vec()), Err(Frame::Overrun)]
        );
    }

    #[test]
    fn gap_after_completed_frame_discards_the_next_one() {
        // the lost word may have been a whole frame or the start of one; either way the
        // first frame after the gap cannot be trusted
        let a = [0x03, 0x3F, 0x3C];
        let c = [0x04, 0x3F, 0x3B];
        let mut asm = FrameAssembler::new();
        asm.gap_after(1);
        let mut words = encode(&a);
        words.extend(encode(&c));
        words.extend(encode(&a));
        assert_eq!(
            frames(&mut asm, &words),
            [Ok(a.to_vec()), Err(Frame::Overrun), Ok(a.to_vec())]
        );
    }

    #[test]
    fn malformed_words_reset_state() {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let mut asm = FrameAssembler::new();
        let words = encode(&[1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(asm.push(words[0], &mut buf), Frame::Partial);
        for bad in [0, 1, 1 << 10, 1 << 31, u32::MAX] {
            assert_eq!(asm.push(bad, &mut buf), Frame::Malformed(bad));
        }
        // a fresh frame decodes from the start of the buffer
        assert_eq!(
            asm.push(encode(&[9, 8, 1])[0], &mut buf),
            Frame::Complete(3)
        );
        assert_eq!(&buf[..3], &[9, 8, 1]);
    }
}
