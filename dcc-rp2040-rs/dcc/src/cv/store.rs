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
#[allow(async_fn_in_trait, reason="This trait is private and doesn't currently require send")]
pub trait Store {
    /// Reads a single configuration variable from the specified CV address (1-indexed).
    async fn read_byte(&self, address: usize) -> Result<u8, Error>;

    /// Reads a range of configuration variables from the specified start CV address (1-indexed).
    async fn read_bytes(&self, address: usize, len: usize) -> Result<&[u8], Error>;

    /// writes a single configuration variable to the specified CV address (1-indexed).
    async fn write_byte(&mut self, address: usize, value: u8) -> Result<(), Error>;

    /// Writes a multi-byte configuration value to the specified CV address (1-indexed)
    ///
    /// If `force` is true, the write is persisted even if [value] is present in the store.
    async fn write_bytes(&mut self, address: usize, value: &[u8], force: bool) -> Result<(), Error>;
}

#[allow(async_fn_in_trait, reason="This trait is private and doesn't currently require send")]
pub trait CvValue: Sized {
    const SIZE: usize;

    async fn from_store(store: &impl Store, address: usize) -> Result<Self, Error>;
}

impl CvValue for u8 {
    const SIZE: usize = size_of::<u8>();

    fn from_store(store: &impl Store, address: usize) -> impl Future<Output = Result<Self, Error>> {
        store.read_byte(address)
    }
}

impl CvValue for u16 {
    const SIZE: usize = size_of::<u16>();

    async fn from_store(store: &impl Store, address: usize) -> Result<Self, Error> {
        let buf = store.read_bytes(address, Self::SIZE).await?;
        buf.try_into()
            .map(u16::from_be_bytes)
            .map_err(|_| Error::Io)
    }
}

impl CvValue for u32 {
    const SIZE: usize = size_of::<u32>();

    async fn from_store(store: &impl Store, address: usize) -> Result<Self, Error> {
        let buf = store.read_bytes(address, Self::SIZE).await?;
        buf.try_into()
            .map(u32::from_be_bytes)
            .map_err(|_| Error::Io)
    }
}

// TODO: add a method for reading decoder configuration as a bitflags struct for easy access
#[allow(async_fn_in_trait, reason="This trait is private and doesn't currently require send")]
pub trait StoreExt {
    async fn read_cv<V: CvValue>(&self, address: usize) -> Result<V, Error>;

    /// Reads the address from the store.
    async fn addr(&self) -> Result<u16, Error>;

    async fn v_start(&self) -> Result<u8, Error>;

    async fn acceleration_rate(&self) -> Result<u8, Error>;

    async fn deceleration_rate(&self) -> Result<u8, Error>;

    async fn v_high(&self) -> Result<u8, Error>;
    async fn v_mid(&self) -> Result<u8, Error>;

    /// Sample time for the PID controller.
    async fn pid_sample_time(&self) -> Result<Duration, Error>;

    /// PID ki term
    async fn pid_ki(&self) -> Result<f32, Error>;

    /// PID kd term
    async fn pid_kd(&self) -> Result<f32, Error>;

    /// Low pass filter time constant.
    async fn pid_filter_tc(&self) -> Result<Duration, Error>;

    /// Position of the end of range1 as a percentage of the
    /// max output
    async fn pid_kp_gain_range1_end(&self) -> Result<f32, Error>;
    async fn pid_kp_y0(&self) -> Result<f32, Error>;

    async fn pid_kp_y1(&self) -> Result<f32, Error>;

    async fn pid_kp_y2(&self) -> Result<f32, Error>;

    /// Feed forward factor for the PID controller.
    async fn pid_k_ff(&self) -> Result<f32, Error>;

    /// Delay between cutting power to the motor and measuring the EMF signal.
    async fn emf_measurement_delay(&self) -> Result<Duration, Error>;

    /// Number of ADC samples taken per EMF measurement (CV61).
    async fn emf_samples(&self) -> Result<u8, Error>;

    async fn emf_l_side_cutoff(&self) -> Result<u8, Error>;
    async fn emf_r_side_cutoff(&self) -> Result<u8, Error>;

    async fn motor_pwm_frequency(&self) -> Result<u32, Error>;

    async fn emf_adc_offset(&self) -> Result<Option<u8>, Error>;

    async fn emf_adc_offset_clear(&mut self) -> Result<(), Error>;

    async fn write_emf_adc_offset(&mut self, offset: u8) -> Result<(), Error>;

    async fn motor_pwm_divider(&self) -> Result<u8, Error>;

    async fn speed_step_period(&self) -> Result<Duration, Error>;

    /// Returns the GPIO PWM enable mask derived from CV_112..CV_115.
    async fn pwm_enabled_mask(&self) -> Result<u32, Error>;

    /// Returns the GPIO output mask for a function index and direction.
    ///
    /// The returned value is a mask representing ALL GPIO pins. Bits that are `1` in the mask
    /// should be enabled when the function is active.
    async fn function_output_mask(
        &self,
        function_index: u8,
        direction: Direction,
    ) -> Result<Option<u32>, Error>;

    /// Returns the PWM configuration for the requested slice.
    ///
    /// Returns None if the slice is invalid.
    async fn pwm_configuration(&self, slice: u8) -> Result<Option<PwmConfig>, Error>;
}

impl<T: Store> StoreExt for T {
    fn read_cv<V: CvValue>(&self, address: usize) -> impl Future<Output = Result<V, Error>> {
        V::from_store(self, address)
    }

    async fn addr(&self) -> Result<u16, Error> {
        // check if we are an extended address
        if 0b00100000 & self.read_byte(DecoderConfiguration as usize).await? != 0 {
            return Ok(read_extended_address(
                self.read_bytes(ExtendedAddressMsb as usize, 2).await?,
            ));
        }

        Ok(self.read_byte(PrimaryAddress as usize).await? as u16)
    }

    fn v_start(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(VStart as usize)
    }

    fn acceleration_rate(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(AccelerationRate as usize)
    }

    fn deceleration_rate(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(DecelerationRate as usize)
    }

    fn v_high(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(VHigh as usize)
    }

    fn v_mid(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(VMid as usize)
    }

    async fn pid_sample_time(&self) -> Result<Duration, Error> {
        self.read_byte(PidSampleTime as usize).await.map(|ms| Duration::from_millis(ms as u64))
    }

    async fn pid_ki(&self) -> Result<f32, Error> {
        self.read_byte(PidKi as usize).await.map(|ki| ki as f32 / 10.0)
    }

    async fn pid_kd(&self) -> Result<f32, Error> {
        self.read_byte(PidKd as usize).await.map(|kd| kd as f32 / 10000.0)
    }

    async fn pid_filter_tc(&self) -> Result<Duration, Error> {
        self.read_byte(PidFilterTc as usize).await.map(|tc| Duration::from_millis(tc as u64))
    }

    async fn pid_kp_gain_range1_end(&self) -> Result<f32, Error> {
        self.read_byte(PidKpGainRange1End as usize).await.map(|gain| gain as f32 / 255.0)
    }

    async fn pid_kp_y0(&self) -> Result<f32, Error> {
        self.read_cv::<u16>(PidKpY0 as usize).await.map(|y0| y0 as f32 / 100.0)
    }

    async fn pid_kp_y1(&self) -> Result<f32, Error> {
        self.read_cv::<u16>(PidKpY1 as usize).await.map(|y1| y1 as f32 / 100.0)
    }

    async fn pid_kp_y2(&self) -> Result<f32, Error> {
        self.read_cv::<u16>(PidKpY2 as usize).await.map(|y2| y2 as f32 / 100.0)
    }

    async fn pid_k_ff(&self) -> Result<f32, Error> {
        self.read_byte(PidFf as usize).await.map(|ff| ff as f32 / 255.0)
    }

    async fn emf_measurement_delay(&self) -> Result<Duration, Error> {
        self.read_byte(EmfMsrDelay as usize).await.map(|us| Duration::from_micros(us as u64))
    }

    fn emf_samples(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(EmfMsrSamples as usize)
    }

    fn emf_l_side_cutoff(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(EmfMsrLowCutoff as usize)
    }

    fn emf_r_side_cutoff(&self) -> impl Future<Output = Result<u8, Error>> {
        self.read_byte(EmfMsrHighCutoff as usize)
    }

    async fn motor_pwm_frequency(&self) -> Result<u32, Error> {
        self.read_byte(MotorPwmFrequency as usize).await.map(|f| f as u32 * 100 + 10_000)
    }

    async fn emf_adc_offset(&self) -> Result<Option<u8>, Error> {
        self.read_byte(EmfAdcOffset as usize).await.map(|offset| (offset != u8::MAX).then_some(offset))
    }

    fn emf_adc_offset_clear(&mut self) -> impl Future<Output = Result<(), Error>> {
        self.write_byte(EmfAdcOffset as usize, u8::MAX)
    }

    fn write_emf_adc_offset(&mut self, offset: u8) -> impl Future<Output = Result<(), Error>> {
        self.write_byte(EmfAdcOffset as usize, offset)
    }

    async fn motor_pwm_divider(&self) -> Result<u8, Error> {
        // ensure the pwm divider is at least 1 so there are no divide by zero errors
        self.read_byte(MotorPwmDivider as usize).await.map(|e| e.max(1))
    }

    async fn speed_step_period(&self) -> Result<Duration, Error> {
        self.read_byte(SpeedStepPeriod as usize).await.map(|ms| Duration::from_millis(ms as u64))
    }

    fn pwm_enabled_mask(&self) -> impl Future<Output = Result<u32, Error>> {
        self.read_cv::<u32>(EnablePwmOutputMask as usize)
    }

    async fn function_output_mask(
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
        self.read_cv::<u32>(cv_start).await.map(Some)
    }

    async fn pwm_configuration(&self, slice: u8) -> Result<Option<PwmConfig>, Error> {
        if slice >= 8 {
            return Ok(None);
        }

        let base = PWM_SLICE_CONFIG_BASE + (slice as usize * PWM_SLICE_CONFIG_STRIDE);

        Ok(Some(PwmConfig {
            wrap: self.read_cv::<u16>(base).await?,
            divider: self.read_byte(base + 2).await?.saturating_add(1),
            a_level: self.read_cv::<u16>(base + 3).await?,
            b_level: self.read_cv::<u16>(base + 5).await?,
        }))
    }
}

pub async fn ensure_populated(store: &mut impl Store) -> Result<(), Error> {
    let primary_addr = store.read_byte(PrimaryAddress as usize).await?;
    trace!("ensure_populated(primary_addr={})", primary_addr);

    if matches!(primary_addr, 0 | u8::MAX) {
        warn!("CV store is empty. Resetting to default values.");
        reset(store).await?;
    }

    Ok(())
}

/// Resets the CV store to the default values.
pub fn reset(store: &mut impl Store) -> impl Future<Output = Result<(), Error>> {
    store.write_bytes(1, DEFAULT_VALUES.as_slice(), true)
}

#[cfg(test)]
mod tests {
    #[cfg(test)]
    extern crate std;
    use super::*;
    use crate::cv::{CV_SIZE, Cv};
    use crate::testing::{MockStore, block_on};

    #[test]
    fn test_addr_primary_address() {
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000); // Extended address bit not set

        assert_eq!(block_on(store.addr()).unwrap(), 42);
    }

    #[test]
    fn test_addr_extended_address() {
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42) // This should be ignored
            .with_extended_address(1234);

        assert_eq!(block_on(store.addr()).unwrap(), 1234);
    }

    #[test]
    fn test_addr_extended_address_min() {
        let store = MockStore::new().with_extended_address(128); // Minimum extended address

        assert_eq!(block_on(store.addr()).unwrap(), 128);
    }

    #[test]
    fn test_addr_extended_address_max() {
        let store = MockStore::new().with_extended_address(16383); // Maximum extended address (0x3FFF)

        assert_eq!(block_on(store.addr()).unwrap(), 16383);
    }

    #[test]
    fn test_addr_primary_address_boundaries() {
        // Test minimum primary address
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 1)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000);

        assert_eq!(block_on(store.addr()).unwrap(), 1);

        // Test maximum primary address
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 127)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00000000);

        assert_eq!(block_on(store.addr()).unwrap(), 127);
    }

    #[test]
    fn test_addr_extended_bit_masking() {
        // Test that only the extended address bit (bit 5) matters in CV29
        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b11011111); // All bits set except extended address bit

        assert_eq!(block_on(store.addr()).unwrap(), 42); // Should use primary address

        let store = MockStore::new()
            .with_cv(Cv::PrimaryAddress as u16, 42)
            .with_cv(Cv::DecoderConfiguration as u16, 0b00100000) // Only extended address bit set
            .with_extended_address(1000);

        assert_eq!(block_on(store.addr()).unwrap(), 1000); // Should use extended address
    }

    #[test]
    fn test_addr_ensure_populated() {
        let mut store = MockStore::new().with_cv(Cv::PrimaryAddress as u16, 0);

        block_on(ensure_populated(&mut store)).unwrap();

        // after a reset we should have the default address of 3
        assert_eq!(block_on(store.addr()), Ok(3));
    }
}
