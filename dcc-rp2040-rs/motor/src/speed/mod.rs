use crate::speed::Mode::{Pid, Startup};
use crate::speed::startup::ComputeResult;
use embassy_time::Duration;

pub mod accel;
mod pid;
mod startup;
pub mod table;

pub use pid::Config as PidConfig;
pub use startup::Config as StartupConfig;

/// When the corrected measured BEMF voltage is above this value, the controller switches to PID mode.
const CONTROLLER_HANDOVER: f32 = 7.5f32;

/// Below this BEMF measured voltage, the motor is considered stalling
/// and we'll revert to startup mode.
const STALL_THRESHOLD: f32 = CONTROLLER_HANDOVER / 2.0;

/// In PID mode, the motor is considered stalled when the corrected measured voltage stays below
/// [STALL_THRESHOLD] for this long. The controller then returns to startup mode to get the motor
/// moving again.
///
/// This is converted to a number of consecutive control steps using the PID sample time (see
/// [max_stall_count]), which assumes [Controller::compute] is called once per sample period.
const STALL_DURATION: Duration = Duration::from_millis(100);

/// The minimum number of consecutive low measurements for a stall, however long the sample time.
///
/// Requiring several consecutive samples keeps single noisy measurements from triggering a
/// restart. With a long sample time, [STALL_DURATION] alone could be as few as one or two samples.
const MIN_STALL_COUNT: u16 = 5;

/// The number of consecutive low measurements that make a stall: [STALL_DURATION] rounded up to
/// whole sample periods, and at least [MIN_STALL_COUNT].
fn max_stall_count(sample_time: Duration) -> u16 {
    let sample_ns = sample_time.as_nanos();
    if sample_ns == 0 {
        // not a valid sample time (the PID controller rejects it); don't divide by zero
        return MIN_STALL_COUNT;
    }

    let samples = STALL_DURATION.as_nanos().div_ceil(sample_ns);
    u16::try_from(samples)
        .unwrap_or(u16::MAX)
        .max(MIN_STALL_COUNT)
}

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

    /// Consecutive PID mode steps with the measurement below [STALL_THRESHOLD].
    stall_count: u16,

    /// Consecutive low steps that make a stall. See [max_stall_count].
    max_stall_count: u16,
}

impl Controller {
    pub fn new(config: Config) -> Result<Self, Error> {
        // validates the PID config (including the sample time) before it is used below
        let pid = pid::Controller::new(config.pid)?;

        Ok(Self {
            config,
            mode: Startup,
            pid,
            startup: startup::Controller::new(config.startup),
            stall_count: 0,
            max_stall_count: max_stall_count(config.pid.sample_time),
        })
    }

    /// Resets the controller.
    ///
    /// Eg: when stopping / changing directions
    pub fn reset(&mut self) {
        self.startup.reset();
        self.pid.reset();
        self.mode = Startup;
        self.stall_count = 0;
    }

    /// Tracks whether the motor has stalled while in PID mode.
    ///
    /// Returns true when the measurement has been below [STALL_THRESHOLD] for
    /// [Self::max_stall_count] consecutive steps. Stall detection is skipped when the setpoint is
    /// at or below [CONTROLLER_HANDOVER]: such a low target is too close to stopped to tell a
    /// stall apart from the PID holding the motor at the setpoint.
    fn update_stall_detection(&mut self, measurement: f32, setpoint: u32) -> bool {
        if measurement < STALL_THRESHOLD && setpoint as f32 > CONTROLLER_HANDOVER {
            self.stall_count = self.stall_count.saturating_add(1);
        } else {
            self.stall_count = 0;
        }

        self.stall_count >= self.max_stall_count
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

        if matches!(self.mode, Pid) && self.update_stall_detection(measurement, setpoint) {
            // The motor stopped turning (eg: it hit a tight spot, or the PID let it slow below
            // the point where it keeps moving). Restart from startup mode, which ramps the output
            // up from the previously learned startup level until the motor moves again.
            debug!(
                "motor stalled: measurement below {} for {} steps, restarting",
                STALL_THRESHOLD, self.max_stall_count
            );
            self.reset();
        }

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
                    self.stall_count = 0;
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

    const OUTPUT_MAX: u16 = 4999;
    const TS: u64 = 5;

    fn controller() -> Controller {
        controller_with_sample_time(Duration::from_millis(TS)).unwrap()
    }

    fn controller_with_sample_time(sample_time: Duration) -> Result<Controller, Error> {
        let pid = pid::Config {
            sample_time,
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
        Controller::new(Config::new(pid, startup, 0.0))
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

    /// Runs the controller through startup until it hands over to PID mode. Returns the time of
    /// the handover step.
    fn hand_over(c: &mut Controller, setpoint: u32) -> u64 {
        let mut t = 0;
        for _ in 0..10 {
            t += TS;
            c.compute(0.0, setpoint, t).unwrap();
        }
        t += TS;
        c.compute(CONTROLLER_HANDOVER + 10.0, setpoint, t).unwrap();
        assert!(matches!(c.mode, Pid));
        t
    }

    #[test]
    fn stall_returns_to_startup_after_consecutive_low_samples() {
        let mut c = controller();
        let setpoint = 100;
        let mut t = hand_over(&mut c, setpoint);

        // The motor stops: one step short of the stall threshold stays in PID mode.
        for _ in 1..c.max_stall_count {
            t += TS;
            c.compute(0.0, setpoint, t).unwrap();
            assert!(matches!(c.mode, Pid));
        }

        // The next low sample is a stall: back to startup, which drives the motor again.
        t += TS;
        let out = c.compute(0.0, setpoint, t).unwrap();
        assert!(matches!(c.mode, Startup));
        assert!(
            out > 0,
            "startup should restart from the learned startup level"
        );
    }

    #[test]
    fn noisy_low_samples_do_not_trigger_stall() {
        let mut c = controller();
        let setpoint = 100;
        let mut t = hand_over(&mut c, setpoint);

        // Mostly-low measurements with an occasional normal one never reach max_stall_count in a
        // row.
        let max_stall_count = u32::from(c.max_stall_count);
        for i in 0..(max_stall_count * 10) {
            t += TS;
            let measurement = if i % (max_stall_count - 1) == 0 {
                setpoint as f32
            } else {
                0.0
            };
            c.compute(measurement, setpoint, t).unwrap();
            assert!(matches!(c.mode, Pid), "stalled at step {i}");
        }
    }

    #[test]
    fn slow_running_motor_below_handover_is_not_a_stall() {
        let mut c = controller();
        // a low setpoint just above the handover threshold, eg: speed step 1
        let setpoint = 8;
        let mut t = hand_over(&mut c, setpoint);

        // The motor runs slowly, below the handover threshold but above the stall threshold: it is
        // turning, so it must not be restarted however long it stays there.
        let slow = (STALL_THRESHOLD + CONTROLLER_HANDOVER) / 2.0;
        for _ in 0..(c.max_stall_count * 5) {
            t += TS;
            c.compute(slow, setpoint, t).unwrap();
        }
        assert!(matches!(c.mode, Pid));
    }

    #[test]
    fn no_stall_detection_when_setpoint_is_in_the_stall_band() {
        let mut c = controller();
        let mut t = hand_over(&mut c, 100);

        // The setpoint is at or below the stall threshold, so a low measurement is on target.
        let setpoint = CONTROLLER_HANDOVER as u32;
        for _ in 0..(c.max_stall_count * 5) {
            t += TS;
            c.compute(0.0, setpoint, t).unwrap();
        }
        assert!(matches!(c.mode, Pid));
    }

    #[test]
    fn stall_count_is_cleared_by_reset_and_handover() {
        let mut c = controller();
        let setpoint = 100;
        let mut t = hand_over(&mut c, setpoint);

        // Nearly stall, then stop the motor (reset) and start again.
        for _ in 1..c.max_stall_count {
            t += TS;
            c.compute(0.0, setpoint, t).unwrap();
        }
        c.reset();
        assert_eq!(c.stall_count, 0);

        // A fresh handover must need a full max_stall_count of low samples again.
        t += TS;
        c.compute(CONTROLLER_HANDOVER + 10.0, setpoint, t).unwrap();
        assert!(matches!(c.mode, Pid));
        t += TS;
        c.compute(0.0, setpoint, t).unwrap();
        assert!(matches!(c.mode, Pid));
    }

    #[test]
    fn stall_window_follows_the_sample_time() {
        // STALL_DURATION (100ms) in whole sample periods
        assert_eq!(max_stall_count(Duration::from_millis(5)), 20);
        assert_eq!(max_stall_count(Duration::from_millis(10)), 10);
        // rounded up, so the window is never shorter than STALL_DURATION
        assert_eq!(max_stall_count(Duration::from_millis(3)), 34);
        assert_eq!(max_stall_count(Duration::from_millis(1)), 100);
    }

    #[test]
    fn stall_window_has_a_minimum_number_of_samples() {
        // a long sample time would otherwise need only 2 (or 1) low samples, which noise can do
        assert_eq!(max_stall_count(Duration::from_millis(50)), MIN_STALL_COUNT);
        assert_eq!(max_stall_count(Duration::from_millis(255)), MIN_STALL_COUNT);
    }

    #[test]
    fn zero_sample_time_does_not_panic() {
        assert_eq!(max_stall_count(Duration::MIN), MIN_STALL_COUNT);
        // the controller rejects it as an invalid config instead of panicking
        assert!(controller_with_sample_time(Duration::MIN).is_err());
    }

    #[test]
    fn stall_after_stall_duration_with_a_different_sample_time() {
        let ts = 10;
        let mut c = controller_with_sample_time(Duration::from_millis(ts)).unwrap();
        let setpoint = 100;

        // hand over at this controller's sample time
        let mut t = 0;
        for _ in 0..10 {
            t += ts;
            c.compute(0.0, setpoint, t).unwrap();
        }
        t += ts;
        c.compute(CONTROLLER_HANDOVER + 10.0, setpoint, t).unwrap();
        assert!(matches!(c.mode, Pid));

        // 100ms of low samples at 10ms is 10 steps: the 9th still runs, the 10th is a stall
        for _ in 1..10 {
            t += ts;
            c.compute(0.0, setpoint, t).unwrap();
            assert!(matches!(c.mode, Pid));
        }
        t += ts;
        c.compute(0.0, setpoint, t).unwrap();
        assert!(matches!(c.mode, Startup));
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
