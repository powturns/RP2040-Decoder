use crate::cv::Cv;
use crate::read_extended_address;

#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    // Error reading / writing the store.
    Io,
}

impl core::fmt::Debug for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(self, f)
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Io => {
                write!(f, "IO error")
            },
        }
    }
}

/// A configuration variable store.
///
/// Implementations of this trait should be optimized for fast reads.
/// Writes occur very infrequently so can be relatively slow.
pub trait Store {
    /// Reads a single configuration variable.
    fn read_cv(&self, address: usize) -> u8;

    /// Reads a range of configuration variables.
    fn read_range(&self, start: usize, len: usize) -> &[u8];

    /// writes a single configuration variable.
    fn write_cv(&mut self, address: usize, value: u8) -> Result<(), Error>;

    fn write_range(&mut self, start: usize, value: &[u8]) -> Result<(), Error>;
}

pub trait StoreExt {
    /// Reads the address from the store.
    fn addr(&self) -> u16;
}

impl <T:Store> StoreExt for T {
    fn addr(&self) -> u16 {
        // check if we are an extended address
        if 0b00100000 & self.read_cv(Cv::DecoderConfiguration as usize) != 0 {
            return read_extended_address(
                self.read_range(Cv::ExtendedAddressMsb as usize, 2)
            );
        }

        self.read_cv(Cv::PrimaryAddress as usize) as u16
    }
}

#[cfg(test)]
mod tests {
    #[cfg(test)]
    extern crate std;
    use super::*;
    use crate::cv::CV_SIZE;

    // Mock Store implementation for testing
    struct MockStore {
        cvs: [u8; CV_SIZE],
    }

    impl MockStore {
        fn new() -> Self {
            MockStore {
                cvs: [0; CV_SIZE],
            }
        }

        fn with_cv(mut self, cv_id: u16, value: u8) -> Self {
            if (cv_id as usize) < CV_SIZE {
                self.cvs[cv_id as usize] = value;
            }
            self
        }

        fn with_extended_address(mut self, address: u16) -> Self {
            // Set extended address bit in CV29
            let cv29 = self.cvs[29] | 0b00100000;
            self.cvs[29] = cv29;

            // Set extended address bytes
            self.cvs[17] = ((address >> 8) & 0x3F) as u8 | 0xC0; // MSB with required bits
            self.cvs[18] = (address & 0xFF) as u8; // LSB
            self
        }
    }

    impl Store for MockStore {
        fn read_cv(&self, cv_id: usize) -> u8 {
            if (cv_id as usize) < CV_SIZE {
                self.cvs[cv_id as usize]
            } else {
                0
            }
        }

        fn read_range(&self, start: usize, len: usize) -> &[u8] {
            let start_idx = start as usize;
            if start_idx < CV_SIZE && start_idx + len <= CV_SIZE {
                &self.cvs[start_idx..start_idx + len]
            } else {
                &[]
            }
        }

        fn write_cv(&mut self, cv_id: usize, value: u8) -> Result<(), Error> {
            if (cv_id as usize) < CV_SIZE {
                self.cvs[cv_id as usize] = value;
                Ok(())
            } else {
                Err(Error::Io)
            }
        }

        fn write_range(&mut self, start: usize, value: &[u8]) -> Result<(), Error> {
            let start_idx = start as usize;
            if start_idx < CV_SIZE && start_idx + value.len() <= CV_SIZE {
                self.cvs[start_idx..start_idx + value.len()].copy_from_slice(value);
                Ok(())
            } else {
                Err(Error::Io)
            }
        }
    }

    #[test]
    fn test_addr_primary_address() {
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000); // Extended address bit not set

        assert_eq!(store.addr(), 42);
    }

    #[test]
    fn test_addr_extended_address() {
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42) // This should be ignored
            .with_extended_address(1234);

        assert_eq!(store.addr(), 1234);
    }

    #[test]
    fn test_addr_extended_address_min() {
        let store = MockStore::new()
            .with_extended_address(128); // Minimum extended address

        assert_eq!(store.addr(), 128);
    }

    #[test]
    fn test_addr_extended_address_max() {
        let store = MockStore::new()
            .with_extended_address(16383); // Maximum extended address (0x3FFF)

        assert_eq!(store.addr(), 16383);
    }

    #[test]
    fn test_addr_primary_address_boundaries() {
        // Test minimum primary address
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 1)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000);

        assert_eq!(store.addr(), 1);

        // Test maximum primary address
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 127)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000);

        assert_eq!(store.addr(), 127);
    }

    #[test]
    fn test_addr_extended_bit_masking() {
        // Test that only the extended address bit (bit 5) matters in CV29
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b11011111); // All bits set except extended address bit

        assert_eq!(store.addr(), 42); // Should use primary address

        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00100000) // Only extended address bit set
            .with_extended_address(1000);

        assert_eq!(store.addr(), 1000); // Should use extended address
    }
}