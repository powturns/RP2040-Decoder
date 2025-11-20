use crate::speed::Mode::{STARTUP, PID};
use crate::speed::startup::ComputeResult;

mod accel;
mod table;
mod pid;
mod startup;

pub use pid::Config as PidConfig;
pub use startup::Config as StartupConfig;


/// When the corrected measured voltage is above this value, the controller switches to PID mode.
const CONTROLLER_HANDOVER:f32 = 7.5f32;

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pid: pid::Config,
    startup: startup::Config,

    /// Offset to correct the measured voltage.
    adc_offset: f32,
}

impl Config {
    pub fn new(
        pid: pid::Config,
        startup: startup::Config,
        adc_offset: f32,
    ) -> Self {
        Self { pid, startup, adc_offset }
    }
}

enum Mode {
    STARTUP,

    /// PID controller is active, with the specified feedforward value.
    PID(f32)
}

pub struct Controller {
    config: Config,
    mode: Mode,
    pid: pid::Controller,
    startup: startup::Controller,
}



impl Controller {

    fn new(config: Config) -> Result<Self, Error> {
        Ok(Self {
            config,
            mode: STARTUP,
            pid: pid::Controller::new(config.pid)?,
            startup: startup::Controller::new(config.startup),
        })
    }

    /// Resets the controller.
    ///
    /// Eg: when stopping / changing directions
    fn reset(&mut self) {
        self.startup.reset();
        self.pid.reset();
        self.mode = STARTUP;
    }

    fn compute(
        &mut self,
       measurement: f32,
       setpoint: u32,
       timestamp_ms: u64,
    ) -> Result<Option<u16>, Error> {
        if setpoint == 0 {
            self.reset();
            return Ok(Some(0));
        }

        let measurement = measurement - self.config.adc_offset;

        match self.mode {
            STARTUP => match self.startup.compute(measurement) {
                ComputeResult::Output(out) => Ok(Some(out)),
                ComputeResult::Handover(ff) => {
                    trace!("Switching to PID mode with feedforward={}", ff);
                    self.mode = PID(ff);
                    Ok(None)
                }
            }
            PID(ff) => {
                let out = self.pid.compute(
                    measurement,
                    setpoint as f32,
                    timestamp_ms,
                    ff,
                )?;

                Ok(Some(out))
            }
        }
    }
}

#[derive(Eq, PartialEq)]
#[cfg_attr(test, derive(core::fmt::Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    PidError(pid::Error),
}

impl From<pid::Error> for Error {
    fn from(e: pid::Error) -> Self {
        Error::PidError(e)
    }
}