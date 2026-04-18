use crate::cv::Cv::*;
use crate::cv::DEFAULT_VALUES;
use crate::read_extended_address;
use core::time::Duration;
use motor::Direction;

// Output configuration CV ranges.
const FUNCTION_MAP_BASE: usize = 257; // CV_257 is F0 forward byte 3
const FUNCTION_MAP_STRIDE: usize = 8; // 4 bytes forward + 4 bytes reverse
const FUNCTION_MAP_DIR_OFFSET: usize = 4; // reverse starts after forward 4 bytes
const PWM_SLICE_CONFIG_BASE: usize = 116; // CV_116..CV_122 for slice 0
const PWM_SLICE_CONFIG_STRIDE: usize = 7; // wrap(2) + divider(1) + levels(4)

#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The address is outside of the range supported by the store.
    InvalidAddress,
    /// Error reading / writing the store.
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
            Error::Io => write!(f, "IO error"),
            Error::InvalidAddress => write!(f, "Invalid Address"),
        }
    }
}

pub struct PwmConfig {
    pub wrap: u16,
    pub divider: u8,
    pub a_level: u16,
    pub b_level: u16,
}

/// A configuration variable store.
///
/// Implementations of this trait should be optimized for fast reads.
/// Writes occur very infrequently so can be relatively slow.
pub trait Store {
    /// Reads a single configuration variable.
    fn read_byte(&self, address: usize) -> Result<u8, Error>;

    /// Reads a range of configuration variables.
    fn read_bytes(&self, start: usize, len: usize) -> Result<&[u8], Error>;

    /// writes a single configuration variable.
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), Error>;

    /// Writes a multi-byte configuration value to the specified address.
    ///
    /// If `force` is true, the write is persisted even if [value] is present in the store.
    fn write_bytes(&mut self, address: usize, value: &[u8], force: bool) -> Result<(), Error>;
}

pub trait CvValue: Sized {
    const SIZE: usize;

    fn from_store(store: &impl Store, address: usize) -> Result<Self, Error>;
}

impl CvValue for u8 {
    const SIZE: usize = size_of::<u8>();

    fn from_store(store: &impl Store, address: usize) -> Result<Self, Error> {
        store.read_byte(address)
    }
}

impl CvValue for u16 {
    const SIZE: usize = size_of::<u16>();

    fn from_store(store: &impl Store, address: usize) -> Result<Self, Error> {
        let buf = store.read_bytes(address, Self::SIZE)?;
        buf.try_into()
            .map(u16::from_be_bytes)
            .map_err(|_| Error::Io)
    }
}

impl CvValue for u32 {
    const SIZE: usize = size_of::<u32>();

    fn from_store(store: &impl Store, address: usize) -> Result<Self, Error> {
        let buf = store.read_bytes(address, Self::SIZE)?;
        buf.try_into()
            .map(u32::from_be_bytes)
            .map_err(|_| Error::Io)
    }
}

pub trait StoreExt {
    fn read_cv<V: CvValue>(&self, address: usize) -> Result<V, Error>;

    /// Reads the address from the store.
    fn addr(&self) -> Result<u16, Error>;

    fn v_start(&self) -> Result<u8, Error>;

    fn acceleration_rate(&self) -> Result<u8, Error>;

    fn deceleration_rate(&self) -> Result<u8, Error>;

    fn v_high(&self) -> Result<u8, Error>;
    fn v_mid(&self) -> Result<u8, Error>;

    /// Sample time for the PID controller.
    fn pid_sample_time(&self) -> Result<Duration, Error>;

    /// PID ki term
    fn pid_ki(&self) -> Result<f32, Error>;

    /// PID kd term
    fn pid_kd(&self) -> Result<f32, Error>;

    /// Low pass filter time constant.
    fn pid_filter_tc(&self) -> Result<Duration, Error>;

    /// Position of the end of range1 as a percentage of the
    /// max output
    fn pid_kp_gain_range1_end(&self) -> Result<f32, Error>;
    fn pid_kp_y0(&self) -> Result<f32, Error>;

    fn pid_kp_y1(&self) -> Result<f32, Error>;

    fn pid_kp_y2(&self) -> Result<f32, Error>;

    /// Feed forward factor for the PID controller.
    fn pid_k_ff(&self) -> Result<f32, Error>;

    /// Delay between cutting power to the motor and measuring the EMF signal.
    fn emf_measurement_delay(&self) -> Result<Duration, Error>;

    fn emf_l_side_cutoff(&self) -> Result<u8, Error>;
    fn emf_r_side_cutoff(&self) -> Result<u8, Error>;

    fn motor_pwm_frequency(&self) -> Result<u32, Error>;

    fn emf_adc_offset(&self) -> Result<Option<u8>, Error>;

    fn emf_adc_offset_clear(&mut self) -> Result<(), Error>;

    fn write_emf_adc_offset(&mut self, offset: u8) -> Result<(), Error>;

    fn motor_pwm_divider(&self) -> Result<u8, Error>;

    fn speed_step_period(&self) -> Result<Duration, Error>;

    /// Returns the GPIO PWM enable mask derived from CV_112..CV_115.
    fn pwm_enabled_mask(&self) -> Result<u32, Error>;

    /// Returns the GPIO output mask for a function index and direction.
    ///
    /// The returned value is a mask representing ALL GPIO pins. Bits that are `1` in the mask
    /// should be enabled when the function is active.
    fn function_output_mask(
        &self,
        function_index: u8,
        direction: Direction,
    ) -> Result<Option<u32>, Error>;

    /// Returns the PWM configuration for the requested slice.
    ///
    /// Returns None if the slice is invalid.
    fn pwm_configuration(&self, slice: u8) -> Result<Option<PwmConfig>, Error>;
}

impl<T: Store> StoreExt for T {
    fn read_cv<V: CvValue>(&self, address: usize) -> Result<V, Error> {
        V::from_store(self, address)
    }

    fn addr(&self) -> Result<u16, Error> {
        // check if we are an extended address
        if 0b00100000 & self.read_byte(DecoderConfiguration as usize)? != 0 {
            return Ok(read_extended_address(
                self.read_bytes(ExtendedAddressMsb as usize, 2)?,
            ));
        }

        Ok(self.read_byte(PrimaryAddress as usize)? as u16)
    }

    fn v_start(&self) -> Result<u8, Error> {
        self.read_byte(VStart as usize)
    }

    fn acceleration_rate(&self) -> Result<u8, Error> {
        self.read_byte(AccelerationRate as usize)
    }

    fn deceleration_rate(&self) -> Result<u8, Error> {
        self.read_byte(DecelerationRate as usize)
    }

    fn v_high(&self) -> Result<u8, Error> {
        self.read_byte(VHigh as usize)
    }

    fn v_mid(&self) -> Result<u8, Error> {
        self.read_byte(VMid as usize)
    }

    fn pid_sample_time(&self) -> Result<Duration, Error> {
        Ok(Duration::from_millis(
            self.read_byte(PidSampleTime as usize)? as u64,
        ))
    }

    fn pid_ki(&self) -> Result<f32, Error> {
        Ok(self.read_byte(PidKi as usize)? as f32 / 10.0)
    }

    fn pid_kd(&self) -> Result<f32, Error> {
        Ok(self.read_byte(PidKd as usize)? as f32 / 10000.0)
    }

    fn pid_filter_tc(&self) -> Result<Duration, Error> {
        Ok(Duration::from_millis(
            self.read_byte(PidFilterTc as usize)? as u64
        ))
    }

    fn pid_kp_gain_range1_end(&self) -> Result<f32, Error> {
        Ok(self.read_byte(PidKpGainRange1End as usize)? as f32 / 255.0)
    }

    fn pid_kp_y0(&self) -> Result<f32, Error> {
        Ok(self.read_cv::<u16>(PidKpY0 as usize)? as f32 / 100.0)
    }

    fn pid_kp_y1(&self) -> Result<f32, Error> {
        Ok(self.read_cv::<u16>(PidKpY1 as usize)? as f32 / 100.0)
    }

    fn pid_kp_y2(&self) -> Result<f32, Error> {
        Ok(self.read_cv::<u16>(PidKpY2 as usize)? as f32 / 100.0)
    }

    fn pid_k_ff(&self) -> Result<f32, Error> {
        Ok(self.read_byte(PidFf as usize)? as f32 / 255.0)
    }

    fn emf_measurement_delay(&self) -> Result<Duration, Error> {
        Ok(Duration::from_micros(
            self.read_byte(EmfMsrDelay as usize)? as u64
        ))
    }

    fn emf_l_side_cutoff(&self) -> Result<u8, Error> {
        self.read_byte(EmfMsrLowCutoff as usize)
    }

    fn emf_r_side_cutoff(&self) -> Result<u8, Error> {
        self.read_byte(EmfMsrHighCutoff as usize)
    }

    fn motor_pwm_frequency(&self) -> Result<u32, Error> {
        Ok(self.read_byte(MotorPwmFrequency as usize)? as u32 * 100 + 10_000)
    }

    fn emf_adc_offset(&self) -> Result<Option<u8>, Error> {
        let offset = self.read_byte(EmfAdcOffset as usize)?;
        if offset != u8::MAX {
            Ok(Some(offset))
        } else {
            Ok(None)
        }
    }

    fn emf_adc_offset_clear(&mut self) -> Result<(), Error> {
        self.write_byte(EmfAdcOffset as usize, u8::MAX)
    }

    fn write_emf_adc_offset(&mut self, offset: u8) -> Result<(), Error> {
        self.write_byte(EmfAdcOffset as usize, offset)
    }

    fn motor_pwm_divider(&self) -> Result<u8, Error> {
        self.read_byte(MotorPwmDivider as usize)
    }

    fn speed_step_period(&self) -> Result<Duration, Error> {
        Ok(Duration::from_millis(
            self.read_byte(SpeedStepPeriod as usize)? as u64,
        ))
    }

    fn pwm_enabled_mask(&self) -> Result<u32, Error> {
        self.read_cv::<u32>(EnablePwmOutputMask as usize)
    }

    fn function_output_mask(
        &self,
        function_index: u8,
        direction: Direction,
    ) -> Result<Option<u32>, Error> {
        if function_index >= 32 {
            return Ok(None);
        }

        let dir_offset = match direction {
            Direction::Forward => 0,
            Direction::Reverse => FUNCTION_MAP_DIR_OFFSET,
        };
        let cv_start =
            FUNCTION_MAP_BASE + (function_index as usize * FUNCTION_MAP_STRIDE) + dir_offset;
        Ok(Some(self.read_cv::<u32>(cv_start)?))
    }

    fn pwm_configuration(&self, slice: u8) -> Result<Option<PwmConfig>, Error> {
        if slice >= 8 {
            return Ok(None);
        }

        let base = PWM_SLICE_CONFIG_BASE + (slice as usize * PWM_SLICE_CONFIG_STRIDE);

        Ok(Some(PwmConfig {
            wrap: self.read_cv::<u16>(base)?,
            divider: self.read_byte(base + 2)?.saturating_add(1),
            a_level: self.read_cv::<u16>(base + 3)?,
            b_level: self.read_cv::<u16>(base + 5)?,
        }))
    }
}

pub fn ensure_populated(store: &mut impl Store) -> Result<(), Error> {
    let primary_addr = store.addr()?;
    trace!("ensure_populated(primary_addr={})", primary_addr);

    if primary_addr == 0 {
        warn!("CV store is empty. Resetting to default values.");
        reset(store)?;
    }

    Ok(())
}

/// Resets the CV store to the default values.
pub fn reset(store: &mut impl Store) -> Result<(), Error> {
    store.write_bytes(1, DEFAULT_VALUES.as_slice(), true)
}

#[cfg(test)]
mod tests {
    #[cfg(test)]
    extern crate std;
    use super::*;
    use crate::cv::{CV_SIZE, Cv};
    use crate::testing::MockStore;

    #[test]
    fn test_addr_primary_address() {
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000); // Extended address bit not set

        assert_eq!(store.addr().unwrap(), 42);
    }

    #[test]
    fn test_addr_extended_address() {
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42) // This should be ignored
            .with_extended_address(1234);

        assert_eq!(store.addr().unwrap(), 1234);
    }

    #[test]
    fn test_addr_extended_address_min() {
        let store = MockStore::new().with_extended_address(128); // Minimum extended address

        assert_eq!(store.addr().unwrap(), 128);
    }

    #[test]
    fn test_addr_extended_address_max() {
        let store = MockStore::new().with_extended_address(16383); // Maximum extended address (0x3FFF)

        assert_eq!(store.addr().unwrap(), 16383);
    }

    #[test]
    fn test_addr_primary_address_boundaries() {
        // Test minimum primary address
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 1)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000);

        assert_eq!(store.addr().unwrap(), 1);

        // Test maximum primary address
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 127)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000);

        assert_eq!(store.addr().unwrap(), 127);
    }

    #[test]
    fn test_addr_extended_bit_masking() {
        // Test that only the extended address bit (bit 5) matters in CV29
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b11011111); // All bits set except extended address bit

        assert_eq!(store.addr().unwrap(), 42); // Should use primary address

        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00100000) // Only extended address bit set
            .with_extended_address(1000);

        assert_eq!(store.addr().unwrap(), 1000); // Should use extended address
    }
}
