use crate::speed::Mode::{Pid, Startup};
use crate::speed::startup::ComputeResult;

pub mod accel;
mod pid;
mod startup;
pub mod table;

pub use pid::Config as PidConfig;
pub use startup::Config as StartupConfig;

/// When the corrected measured voltage is above this value, the controller switches to PID mode.
const CONTROLLER_HANDOVER: f32 = 7.5f32;

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pid: pid::Config,
    startup: startup::Config,

    /// Offset to correct the measured voltage.
    adc_offset: f32,
}

impl Config {
    pub fn new(pid: pid::Config, startup: startup::Config, adc_offset: f32) -> Self {
        Self {
            pid,
            startup,
            adc_offset,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Mode {
    Startup,

    /// PID controller is active, with the specified feedforward value.
    Pid(f32),
}

/// A unified speed controller that uses the startup controller before the motor is moving
/// then switches to the PID controller after it is running.
///
/// During the startup phase, the controller doesn't consider the target setpoint (from the speed table);
/// its only goal is to get the motor moving quickly. Once the motor starts to move, the setpoint is
/// considered.
///
/// This controller uses back emf for the target and measured values and calculates a PWM
/// level for the output.
pub struct Controller {
    config: Config,
    mode: Mode,
    pid: pid::Controller,
    startup: startup::Controller,
}

impl Controller {
    pub fn new(config: Config) -> Result<Self, Error> {
        Ok(Self {
            config,
            mode: Startup,
            pid: pid::Controller::new(config.pid)?,
            startup: startup::Controller::new(config.startup),
        })
    }

    /// Resets the controller.
    ///
    /// Eg: when stopping / changing directions
    pub fn reset(&mut self) {
        self.startup.reset();
        self.pid.reset();
        self.mode = Startup;
    }

    pub fn compute(
        &mut self,
        measurement: f32,
        setpoint: u32,
        timestamp_ms: u64,
    ) -> Result<Option<u16>, Error> {
        if setpoint == 0 {
            self.reset();
            return Ok(Some(0));
        }

        let measurement = (measurement - self.config.adc_offset).max(0.0); // TODO: should we actually clamp this to zero?

        if cfg!(feature = "verbose") {
            trace!("compute(mode={} measurement={} setpoint={})", self.mode, measurement, setpoint);
        }

        match self.mode {
            Startup => match self.startup.compute(measurement) {
                ComputeResult::Output(out) => {
                    trace!("startup(measurement={} setpoint={}) -> output={}", measurement, setpoint, out);
                    Ok(Some(out))
                },
                ComputeResult::Handover(ff) => {
                    trace!("Switching to PID mode with feedforward={}", ff);
                    self.mode = Pid(ff);
                    Ok(None)
                }
            },
            Pid(ff) => {
                let out = self
                    .pid
                    .compute(measurement, setpoint as f32, timestamp_ms, ff)?;

                trace!("pid(measurement={} setpoint={}) -> output={}", measurement, setpoint, out);

                Ok(Some(out))
            }
        }
    }
}

#[derive(Eq, PartialEq)]
#[cfg_attr(test, derive(Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    PidError(pid::Error),
}

impl From<pid::Error> for Error {
    fn from(e: pid::Error) -> Self {
        Error::PidError(e)
    }
}
