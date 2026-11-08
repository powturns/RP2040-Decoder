extern crate alloc;

use crate::Timer;
use crate::cv::CV_SIZE;
use crate::cv::store::{Error, Store};
use crate::transport::packet::Packet;
use alloc::format;
use core::any::type_name;
use heapless::Vec;

extern crate std;

// Mock Store implementation for testing
pub(crate) struct MockStore {
    cvs: [u8; CV_SIZE],
}

impl MockStore {
    pub(crate) fn new() -> Self {
        MockStore { cvs: [0; CV_SIZE] }
    }

    pub(crate) fn with_cv(mut self, cv_id: u16, value: u8) -> Self {
        let idx = cv_id as usize - 1;

        if idx < CV_SIZE {
            self.cvs[idx] = value;
        }
        self
    }

    pub(crate) fn with_extended_address(mut self, address: u16) -> Self {
        // Set extended address bit in CV29 (remember CVS are 1-indexed, but internally 0 indexed)
        let cv29 = self.cvs[29 - 1] | 0b00100000;
        self.cvs[29 - 1] = cv29;

        // Set extended address bytes
        self.cvs[17 - 1] = ((address >> 8) & 0x3F) as u8 | 0xC0; // MSB with required bits
        self.cvs[18 - 1] = (address & 0xFF) as u8; // LSB
        self
    }
}

impl Store for MockStore {
    fn read_byte(&self, cv_id: usize) -> Result<u8, Error> {
        let idx = cv_id - 1;

        if (idx) < CV_SIZE {
            Ok(self.cvs[idx])
        } else {
            Ok(0)
        }
    }

    fn read_bytes(&self, start: usize, len: usize) -> Result<&[u8], Error> {
        let start_idx = start - 1;
        if start_idx < CV_SIZE && start_idx + len <= CV_SIZE {
            Ok(&self.cvs[start_idx..start_idx + len])
        } else {
            Ok(&[])
        }
    }

    fn write_byte(&mut self, cv_id: usize, value: u8) -> Result<(), Error> {
        let idx = cv_id - 1;

        if idx < CV_SIZE {
            self.cvs[idx] = value;
            Ok(())
        } else {
            Err(Error::Io)
        }
    }

    fn write_bytes(&mut self, start: usize, value: &[u8], force: bool) -> Result<(), Error> {
        let start_idx = start - 1;
        if start_idx < CV_SIZE && start_idx + value.len() <= CV_SIZE {
            self.cvs[start_idx..start_idx + value.len()].copy_from_slice(value);
            Ok(())
        } else {
            Err(Error::Io)
        }
    }
}

// Mock Timer for testing
pub(crate) struct MockTimer {
    pub(crate) running: bool,
    elapsed: Option<usize>,
}

impl MockTimer {
    pub(crate) fn new() -> Self {
        Self {
            running: false,
            elapsed: None,
        }
    }

    pub(crate) fn set_elapsed(&mut self, elapsed: usize) {
        self.elapsed = Some(elapsed);
    }
}

impl Timer for MockTimer {
    fn stop(&mut self) {
        self.running = false;
        self.elapsed = None;
    }

    fn start(&mut self) {
        self.running = true;
    }

    fn elapsed_ms(&self) -> Option<usize> {
        if self.running { self.elapsed } else { None }
    }
}

pub(crate) fn pkt(bytes: &[u8]) -> Packet {
    let data = Vec::from_slice(bytes).expect(&format!(
        "packet too large for: {}",
        type_name::<Vec<u8, 6>>()
    ));

    Packet { data }
}
