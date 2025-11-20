use core::time::Duration;
use crate::cv::Cv;
use crate::cv::Cv::*;
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
    fn read_byte(&self, address: usize) -> u8;

    /// Reads a range of configuration variables.
    fn read_bytes(&self, start: usize, len: usize) -> &[u8];

    /// writes a single configuration variable.
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), Error>;

    fn write_bytes(&mut self, start: usize, value: &[u8]) -> Result<(), Error>;
}

pub trait CvValue {
    const SIZE: usize;

    fn from_store(store: &impl Store, address: usize) -> Self;
}

impl CvValue for u8 {
    const SIZE: usize = size_of::<u8>();

    fn from_store(store: &impl Store, address: usize) -> Self {
        store.read_byte(address)
    }
}

impl CvValue for u16 {
    const SIZE: usize = size_of::<u16>();

    fn from_store(store: &impl Store, address: usize) -> Self {
        let buf = store.read_bytes(address, Self::SIZE);
        u16::from_be_bytes(buf.try_into().expect("Store returned insufficient bytes"))
    }
}

impl CvValue for u32 {
    const SIZE: usize = size_of::<u32>();
    fn from_store(store: &impl Store, address: usize) -> Self {
        let buf = store.read_bytes(address, Self::SIZE);
        u32::from_be_bytes(buf.try_into().expect("Store returned insufficient bytes"))
    }
}

pub trait StoreExt {
    fn read_cv<V:CvValue>(&self, address: usize) -> V;

    /// Reads the address from the store.
    fn addr(&self) -> u16;

    fn v_start(&self) -> u8;
    fn v_high(&self) -> u8;
    fn v_mid(&self) -> u8;

    /// Sample time for the PID controller.
    fn pid_sample_time(&self) -> Duration;

    /// PID ki term
    fn pid_ki(&self) -> f32;

    /// PID kd term
    fn pid_kd(&self) -> f32;

    /// Low pass filter time constant.
    fn pid_filter_tc(&self) -> Duration;

    /// Position of the end of range1 as a percentage of the
    /// max output
    fn pid_kp_gain_range1_end(&self) -> f32;
    fn pid_kp_y0(&self) -> f32;

    fn pid_kp_y1(&self) -> f32;

    fn pid_kp_y2(&self) -> f32;

    /// Feed forward factor for the PID controller.
    fn pid_k_ff(&self) -> f32;

    fn emf_l_side_cutoff(&self) -> u8;
    fn emf_r_side_cutoff(&self) -> u8;

    fn motor_pwm_frequency(&self) -> u32;

    fn emf_adc_offset(&self) -> Option<u8>;
    fn write_emf_adc_offset(&mut self, offset: u8) -> Result<(), Error>;

    fn motor_pwm_divider(&self) -> u8;
}

impl <T:Store> StoreExt for T {
    fn read_cv<V: CvValue>(&self, address: usize) -> V {
        V::from_store(self, address)
    }

    fn addr(&self) -> u16 {
        // check if we are an extended address
        if 0b00100000 & self.read_byte(DecoderConfiguration as usize) != 0 {
            return read_extended_address(
                self.read_bytes(ExtendedAddressMsb as usize, 2)
            );
        }

        self.read_byte(PrimaryAddress as usize) as u16
    }

    fn v_start(&self) -> u8 {
        self.read_byte(VStart as usize)
    }

    fn v_high(&self) -> u8 {
        self.read_byte(VHigh as usize)
    }

    fn v_mid(&self) -> u8 {
        self.read_byte(VMid as usize)
    }

    fn pid_sample_time(&self) -> Duration {
        Duration::from_millis(self.read_byte(PidSampleTime as usize) as u64)
    }

    fn pid_ki(&self) -> f32 {
        self.read_byte(PidKi as usize) as f32 / 10.0
    }

    fn pid_kd(&self) -> f32 {
        self.read_byte(PidKd as usize) as f32 / 10000.0
    }

    fn pid_filter_tc(&self) -> Duration {
        Duration::from_millis(self.read_byte(PidFilterTc as usize) as u64)
    }

    fn pid_kp_gain_range1_end(&self) -> f32 {
        self.read_byte(PidKpGainRange1End as usize) as f32 / 255.0
    }

    fn pid_kp_y0(&self) -> f32 {
        self.read_cv::<u16>(PidKpY0 as usize) as f32 / 100.0
    }

    fn pid_kp_y1(&self) -> f32 {
        self.read_cv::<u16>(PidKpY1 as usize) as f32 / 100.0
    }

    fn pid_kp_y2(&self) -> f32 {
        self.read_cv::<u16>(PidKpY2 as usize) as f32 / 100.0
    }

    fn pid_k_ff(&self) -> f32 {
        self.read_byte(PidFf as usize) as f32 / 255.0
    }

    fn emf_l_side_cutoff(&self) -> u8 {
        self.read_byte(EmfMsrLowCutoff as usize)
    }

    fn emf_r_side_cutoff(&self) -> u8 {
        self.read_byte(EmfMsrHighCutoff as usize)
    }

    fn motor_pwm_frequency(&self) -> u32 {
        self.read_byte(MotorPwmFrequency as usize) as u32 * 100 + 1000
    }

    fn emf_adc_offset(&self) -> Option<u8> {
        let offset = self.read_byte(EmfAdcOffset as usize);
        if offset != u8::MAX {
            Some(offset)
        } else {
            None
        }
    }

    fn write_emf_adc_offset(&mut self, offset: u8) -> Result<(), Error> {
        self.write_byte(EmfAdcOffset as usize, offset)
    }

    fn motor_pwm_divider(&self) -> u8 {
        self.read_byte(MotorPwmDivider as usize)
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
        fn read_byte(&self, cv_id: usize) -> u8 {
            if (cv_id as usize) < CV_SIZE {
                self.cvs[cv_id as usize]
            } else {
                0
            }
        }

        fn read_bytes(&self, start: usize, len: usize) -> &[u8] {
            let start_idx = start as usize;
            if start_idx < CV_SIZE && start_idx + len <= CV_SIZE {
                &self.cvs[start_idx..start_idx + len]
            } else {
                &[]
            }
        }

        fn write_byte(&mut self, cv_id: usize, value: u8) -> Result<(), Error> {
            if (cv_id as usize) < CV_SIZE {
                self.cvs[cv_id as usize] = value;
                Ok(())
            } else {
                Err(Error::Io)
            }
        }

        fn write_bytes(&mut self, start: usize, value: &[u8]) -> Result<(), Error> {
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