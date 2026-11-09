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

    /// PID controller is active. Its integral was seeded with the initial output at handover.
    Pid,
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

    /// Compute the PID output from the measured back EMF and current setpoint.
    ///
    /// `timestamp_ms` is the *scheduled* time of this control step, from a fixed-rate schedule
    /// with the configured PID sample time: it must never decrease, and consecutive steps must be
    /// at least one sample period apart.
    ///
    /// The PID controller is a fixed-rate discrete controller: it always assumes one sample period
    /// between steps and only uses the timestamps to skip steps that come too soon. Passing the
    /// time a step happened to run (rather than its scheduled time) would let jitter skip steps.
    pub fn compute(
        &mut self,
        measurement: f32,
        setpoint: u32,
        timestamp_ms: u64,
    ) -> Result<u16, Error> {
        if setpoint == 0 {
            self.reset();
            return Ok(0);
        }

        let measurement = (measurement - self.config.adc_offset).max(0.0); // TODO: should we actually clamp this to zero?

        if cfg!(feature = "verbose") {
            trace!(
                "compute(mode={} measurement={} setpoint={})",
                self.mode, measurement, setpoint
            );
        }

        static mut ITERATION_COUNT: u32 = 0;
        const SAMPLE_RATE: u32 = 100;
        let count = unsafe {
            ITERATION_COUNT = ITERATION_COUNT.wrapping_add(1);

            ITERATION_COUNT
        };

        match self.mode {
            Startup => match self.startup.compute(measurement) {
                ComputeResult::Output(out) => {
                    if count % SAMPLE_RATE == 0 {
                        trace!("startup(measurement={}) -> output={}", measurement, out);
                    }
                    Ok(out)
                }
                ComputeResult::Handover {
                    initial_output,
                    output,
                } => {
                    trace!("Switching to PID mode, initial output={}", initial_output);
                    // bumpless transfer: the PID continues from the initial output
                    self.pid.seed(measurement, initial_output, timestamp_ms);
                    self.mode = Pid;
                    Ok(output)
                }
            },
            Pid => {
                let out = self.pid.compute(measurement, setpoint as f32, timestamp_ms);

                if count % SAMPLE_RATE == 0 {
                    trace!(
                        "pid(measurement={} setpoint={}) -> output={}",
                        measurement, setpoint, out
                    );
                }

                Ok(out)
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

#[cfg(test)]
mod tests {
    use super::*;
    use core::time::Duration;

    const OUTPUT_MAX: u16 = 4999;
    const TS: u64 = 5;

    fn controller() -> Controller {
        let pid = pid::Config {
            sample_time: Duration::from_millis(TS),
            filter_tc: Duration::from_millis(10),
            ki: 2.5,
            kd: 0.005,
            output_max: OUTPUT_MAX,
            kp_gain_range1_end: 13.0 / 255.0,
            kp_y0: 20.0,
            kp_y1: 2.5,
            kp_y2: 1.5,
            max_setpoint: 400.0,
        };
        let startup = startup::Config {
            output_max: OUTPUT_MAX,
            pid_ff: 0.9,
        };
        Controller::new(Config::new(pid, startup, 0.0)).unwrap()
    }

    #[test]
    fn handover_continues_from_startup_output() {
        let mut c = controller();
        // the measurement the motor reaches at handover; use it as the setpoint so the PID
        // error is zero after handover
        let moving_bemf = 10.0;
        let setpoint = moving_bemf as u32;

        // Ramp in startup mode until the motor moves.
        let mut t = 0;
        let mut last_startup = 0;
        for _ in 0..10 {
            t += TS;
            last_startup = c.compute(0.0, setpoint, t).unwrap();
        }
        assert!(last_startup > 0);

        // The motor starts moving: handover. The startup output is held for this step.
        t += TS;
        let handover_output = c.compute(moving_bemf, setpoint, t).unwrap();
        assert!(matches!(c.mode, Pid));
        assert!(handover_output > 0);

        // With zero error and an unchanged measurement, the PID output is the initial output
        // (CV47 fraction of the handover output), with no kick from P, I or D.
        t += TS;
        let expected = (0.9 * handover_output as f32) as u16;
        let out = c.compute(moving_bemf, setpoint, t).unwrap();
        assert!(
            out.abs_diff(expected) <= 1,
            "pid output {out} should continue from the initial output {expected}"
        );
    }

    #[test]
    fn reset_returns_to_startup() {
        let mut c = controller();
        c.compute(CONTROLLER_HANDOVER + 1.0, 100, TS).unwrap();
        assert!(matches!(c.mode, Pid));

        c.reset();
        assert!(matches!(c.mode, Startup));
    }
}
