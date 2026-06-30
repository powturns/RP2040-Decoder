pub mod handler;
use crate::MotorResources;
use embassy_rp::gpio::Pull;

use embassy_rp::adc;
use embassy_rp::adc::{Adc, Async, Channel};
use embassy_rp::pwm;
use embassy_rp::pwm::{Pwm, PwmError, PwmOutput};
use embassy_rp::dma;
use embassy_time::{Duration, Timer};
use embedded_hal::pwm::SetDutyCycle;
use math::filtered_mean;

use motor::{Direction, VelocitySetpoint};

const ADC_CALIBRATION_ITERATIONS: usize = 8192;

/// Maximum number of ADC samples taken per EMF measurement. The sample count comes from
/// CV61, which is a `u8`, so 255 covers the entire configurable range.
const MAX_EMF_SAMPLES: usize = u8::MAX as usize;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Pwm(#[cfg_attr(feature = "defmt", defmt(Debug2Format))] pwm::PwmError),
    Adc(adc::Error),
}

impl From<pwm::PwmError> for Error {
    fn from(value: PwmError) -> Self {
        Self::Pwm(value)
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub pwm_max_output: u16,

    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub pwm_clk_divider: fixed::FixedU16<fixed::types::extra::U4>,

    pub emf_measurement_delay: Duration,

    /// Number of ADC samples per EMF measurement. Clamped to [1, MAX_SAMPLES].
    pub emf_samples: u8,

    /// Samples discarded from the high end of the sorted emf measurement.
    pub emf_l_cutoff: u8,

    /// Samples discarded from the high end of the sorted emf measurement.
    pub emf_r_cutoff: u8,
}

pub struct RpMotorController {
    config: Config,
    dir: Direction,
    fwd: DirectionControl,
    rev: DirectionControl,
    adc: Adc<'static, Async>,
    pub dma: dma::Channel<'static>,
    ack_dir: Direction,
    /// Fixed-capacity buffer for EMF samples, reused across measurements. Only the first
    /// `config.emf_samples` entries are used on any given measurement.
    emf_buf: [u16; MAX_EMF_SAMPLES],
}

impl RpMotorController {
    pub fn new(
        config: Config,
        resources: MotorResources,
        adc: Adc<'static, Async>,
        dma: dma::Channel<'static>,
    ) -> Self {
        let mut pwm_config = pwm::Config::default();
        pwm_config.compare_a = 0;
        pwm_config.compare_b = 0;
        pwm_config.top = config.pwm_max_output; // FIXME: c code subtracts 1
        pwm_config.divider = config.pwm_clk_divider;

        let pwm = Pwm::new_output_ab(
            resources.pwm_slice,
            resources.rev_pin,
            resources.fwd_pin,
            pwm_config,
        );

        let (rev_out, fwd_out) = pwm.split();

        let fwd = DirectionControl::new(
            fwd_out.unwrap(),
            Channel::new_pin(resources.fwd_emf_in, Pull::None),
        );

        let rev = DirectionControl::new(
            rev_out.unwrap(),
            Channel::new_pin(resources.rev_emf_in, Pull::None),
        );

        Self {
            config,
            dir: Direction::Forward,
            fwd,
            rev,
            adc,
            dma,
            ack_dir: Direction::Forward,
            emf_buf: [0; MAX_EMF_SAMPLES],
        }
    }

    /// Measure the back emf of the motor.
    async fn measure(&mut self) -> Result<f32, Error> {
        // FIXME: restore the levels after the measurement?
        self.fwd.stop()?;
        self.rev.stop()?;

        // Number of samples to take, clamped to the fixed-capacity buffer (CV61).
        let n = (self.config.emf_samples as usize).clamp(1, MAX_EMF_SAMPLES);

        // Samples discarded from each end of the sorted measurement (CV63 / CV64). Guard
        // against pathological CVs that would otherwise discard every sample.
        let (l_cutoff, r_cutoff) = {
            let l = self.config.emf_l_cutoff as usize;
            let r = self.config.emf_r_cutoff as usize;
            if l + r < n { (l, r) } else { (0, 0) }
        };

        let buf = &mut self.emf_buf[..n];

        Timer::after(self.config.emf_measurement_delay).await;

        match self.dir {
            Direction::Forward => {
                self.fwd
                    .measure_emf(&mut self.adc, buf, &mut self.dma)
                    .await?
            }
            Direction::Reverse => {
                self.rev
                    .measure_emf(&mut self.adc, buf, &mut self.dma)
                    .await?
            }
        }

        buf.sort_unstable();

        let filtered_cnt = n - (l_cutoff + r_cutoff);

        let sum: u32 = buf
            .iter()
            .skip(l_cutoff)
            .take(filtered_cnt)
            .map(|&x| x as u32)
            .sum();

        Ok(sum as f32 / filtered_cnt as f32)
    }
}

impl Controller for RpMotorController {
    type Error = Error;
    /// Measures the back emf ADC values when the motor isn't running.
    async fn measure_adc_offset(&mut self) -> Result<u16, Error> {
        let buf = &mut [0; ADC_CALIBRATION_ITERATIONS];

        self.stop()?;

        Timer::after_secs(1).await;

        debug!("Measuring ADC offset in reverse direction. n={}", buf.len());
        let _ = &mut self
            .rev
            .measure_emf(&mut self.adc, buf, &mut self.dma)
            .await?;
        let offset_avg_rev = filtered_mean(buf, 2).unwrap_or(0);
        debug!("offset_avg_rev={}", offset_avg_rev);

        Timer::after_secs(1).await;
        debug!("Measuring ADC offset in forward direction. n={}", buf.len());
        let _ = &mut self
            .fwd
            .measure_emf(&mut self.adc, buf, &mut self.dma)
            .await?;
        let offset_avg_fwd = filtered_mean(buf, 2).unwrap_or(0);
        debug!("offset_avg_fwd={}", offset_avg_fwd);

        Ok(((offset_avg_fwd as u32 + offset_avg_rev as u32) / 2) as u16)
    }
    /// Acknowledge a CV rd/wr instruction by pulsing the motor in both directions
    async fn acknowledge_cv(&mut self) -> Result<(), Error> {
        self.stop()?;

        let (next_direction, ctrl)  = match self.ack_dir {
            Direction::Forward => (Direction::Reverse, &mut self.fwd),
            Direction::Reverse => (Direction::Forward, &mut self.rev),
        };
        self.ack_dir = next_direction;

        // FIXME: we want to ack but we dont want to trip the programming track current
        ctrl.set_output_percent(50)?;
        Timer::after_millis(6).await;
        ctrl.stop()?;

        Ok(())
    }
    fn stop(&mut self) -> Result<(), Error> {
        self.fwd.stop()?;
        self.rev.stop()?;
        Ok(())
    }

    fn set_output_level(&mut self, pwm_level: u16, direction: Direction) -> Result<(), Error> {
        self.dir = direction;

        match direction {
            Direction::Forward => {
                self.rev.stop()?;
                self.fwd.set_output(pwm_level)?;
            }
            Direction::Reverse => {
                self.fwd.stop()?;
                self.rev.set_output(pwm_level)?;
            }
        }

        Ok(())
    }
}

pub trait Controller {
    type Error;

    /// Measures the back emf ADC values when the motor isn't running.
    async fn measure_adc_offset(&mut self) -> Result<u16, Self::Error>;
    /// Acknowledge a CV rd/wr instruction by pulsing the motor in both directions
    async fn acknowledge_cv(&mut self) -> Result<(), Self::Error>;
    fn stop(&mut self) -> Result<(), Self::Error>;
    fn set_output_level(&mut self, level: u16, direction: Direction) -> Result<(), Error>;
}

struct DirectionControl {
    motor_pwm: PwmOutput<'static>,
    emf: Channel<'static>,
}

impl DirectionControl {
    pub fn new(motor_pwm: PwmOutput<'static>, emf: Channel<'static>) -> Self {
        Self { motor_pwm, emf }
    }

    /// Measures the emf using the [adc] into the [buf].
    async fn measure_emf(
        &mut self,
        adc: &mut Adc<'_, Async>,
        buf: &mut [u16],
        dma: &mut dma::Channel<'_>,
    ) -> Result<(), Error> {
        adc.read_many(
            &mut self.emf,
            buf,
            0, // sample at the full 500,000 samples per second
            dma,
        )
        .await
        .map_err(Error::Adc)?;

        Ok(())
    }

    fn set_output(&mut self, duty_cycle: u16) -> Result<(), Error> {
        Ok(self.motor_pwm.set_duty_cycle(duty_cycle)?)
    }

    fn set_output_percent(&mut self, percent: u8) -> Result<(), Error> {
        debug_assert!(percent <= 100);
        Ok(self.motor_pwm.set_duty_cycle_percent(percent.min(100))?)
    }

    fn stop(&mut self) -> Result<(), Error> {
        Ok(self.motor_pwm.set_duty_cycle_fully_off()?)
    }
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    AcknowledgeCv,

    Reset,

    /// A 128 speed step velocity setpoint.
    SetVelocity128(VelocitySetpoint),
}
