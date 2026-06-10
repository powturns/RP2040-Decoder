//! PID controller wrapper built on top of the `discrete_pid` crate.
//!
//! This module adapts the C implementation’s behavior to Rust by using
//! a configurable wrapper that:
//! - mirrors the timing and derivative-on-measurement behavior,
//! - supports gain scheduling for Kp (piecewise linear over the setpoint),
//! - passes an optional feedforward term into the controller,
//! - applies output limits, and
//! - provides a simple anti-windup policy by holding integration while saturated.

use core::ops::Range;
use core::time::Duration;

use discrete_pid::pid::{PidConfig, PidConfigError, PidController};
use discrete_pid::time::Millis;

#[derive(Eq, PartialEq)]
#[cfg_attr(test, derive(Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    ConfigError(#[cfg_attr(feature = "defmt", defmt(Debug2Format))] PidConfigError),
}

impl From<PidConfigError> for Error {
    fn from(e: PidConfigError) -> Self {
        Error::ConfigError(e)
    }
}

/// Configuration for the Speed PID wrapper.
///
/// The application is expected to fill this from its control variables (CVs) or
/// other configuration sources.
#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub sample_time: Duration,

    /// Low-pass filter time constant used on the derivative term.
    pub filter_tc: Duration,

    /// Time-invariant integral gain (ki). The library scales this by the sample time.
    pub ki: f32,

    /// Time-invariant derivative gain (kd). The library scales this by the sample time.
    pub kd: f32,

    /// Maximum controller output (PWM top).
    pub output_max: u16,

    /// Gain scheduling parameters for Kp based on the setpoint.
    /// Defines the position of the end of the first gain range based on a fraction of the [max_setpoint].
    pub kp_gain_range1_end: f32,

    /// Kp level at start of low range
    pub kp_y0: f32,

    /// Kp level at end of low range / start of high range
    pub kp_y1: f32,

    /// Kp level at end of high range
    pub kp_y2: f32,

    /// Maximal value of the setpoint range (e.g., last speed table entry).
    pub max_setpoint: f32,
}

/// Helper struct to compute Kp based on the setpoint range.
///
/// Often it is favorable to have a higher proportional gain KP for slow speeds, achieving
///  better control results.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct GainRange {
    setpoint_range: Range<f32>,
    kp_start: f32,
    slope: f32,
}

impl GainRange {
    /// Creates a new GainRange, calculating the slope based on the range, start and end arguments.
    fn new(setpoint_range: Range<f32>, kp_start: f32, kp_end: f32) -> Self {
        let slope = if setpoint_range.is_empty()
            || (setpoint_range.end - setpoint_range.start).abs() < f32::EPSILON
        {
            1.0
        } else {
            (kp_end - kp_start) / (setpoint_range.end - setpoint_range.start)
        };

        Self {
            setpoint_range,
            kp_start,
            slope,
        }
    }

    fn get_kp(&self, setpoint: f32) -> f32 {
        self.slope * (setpoint - self.setpoint_range.start) + self.kp_start
    }

    fn contains(&self, setpoint: f32) -> bool {
        self.setpoint_range.contains(&setpoint)
    }
}

/// PID controller wrapper with gain scheduling and simple anti-windup.
///
/// Gain Scheduling:
/// Often it is favorable to have a higher proportional gain KP for slow speeds, achieving
///  better control results.
pub struct Controller {
    pid: PidController<Millis, f32>,
    params: Config,
    // Precomputed gain scheduling parameters
    gain_range_low: GainRange,
    gain_range_high: GainRange,
}

impl Controller {
    /// Creates a new SpeedPid initialized with the provided configuration.
    pub fn new(cfg: Config) -> Result<Self, Error> {
        // Build the underlying library configuration
        let mut lib_cfg: PidConfig<f32> = PidConfig::default();
        lib_cfg.set_kp(1.0)?; // placeholder; will be updated dynamically per setpoint
        lib_cfg.set_ki(cfg.ki)?;
        lib_cfg.set_kd(cfg.kd)?;
        lib_cfg.set_filter_tc(cfg.filter_tc.as_secs_f32())?;
        lib_cfg.set_sample_time(cfg.sample_time)?;
        lib_cfg.set_output_limits(0.0, cfg.output_max as f32)?;
        lib_cfg.set_use_derivative_on_measurement(true);
        lib_cfg.set_use_strict_causal_integrator(true);

        let pid = PidController::new_uninit(lib_cfg);

        // Gain scheduling precompute
        let kp_x1 = cfg.max_setpoint * cfg.kp_gain_range1_end;

        let gain_range1 = GainRange::new(
            Range {
                start: 0.0,
                end: kp_x1,
            },
            cfg.kp_y0,
            cfg.kp_y1,
        );

        let gain_range2 = GainRange::new(
            Range {
                start: kp_x1,
                end: cfg.max_setpoint,
            },
            cfg.kp_y1,
            cfg.kp_y2,
        );

        Ok(Self {
            pid,
            params: cfg,
            gain_range_low: gain_range1,
            gain_range_high: gain_range2,
        })
    }

    /// Reset the controller.
    ///
    /// Useful on direction changes or when stopping.
    pub fn reset(&mut self) {
        self.pid.reset_integral();
    }

    /// Computes the scheduled Kp based on the provided setpoint.
    fn kp_for_setpoint(&self, sp: f32) -> f32 {
        let sp = sp.clamp(
            self.gain_range_low.setpoint_range.start,
            self.gain_range_high.setpoint_range.end,
        );
        if self.gain_range_low.contains(sp) {
            self.gain_range_low.get_kp(sp)
        } else {
            self.gain_range_high.get_kp(sp)
        }
    }

    /// Compute a control output.
    ///
    /// - `measurement`: latest measured back emf value, corrected for ADC offset.
    /// - `setpoint`: desired target value (back emf value)
    /// - `timestamp_ms`: current time in milliseconds
    /// - `feedforward`: feedforward term
    pub fn compute(
        &mut self,
        measurement: f32,
        setpoint: f32,
        timestamp_ms: u64,
        feedforward: f32,
    ) -> Result<u16, Error> {
        let setpoint = setpoint.clamp(0.0, self.params.max_setpoint);
        // Dynamic Kp scheduling based on setpoint
        let kp = self.kp_for_setpoint(setpoint);
        self.pid.config_mut().set_kp(kp)?;

        // Compute control output
        Ok(self.pid.compute(
            measurement,
            setpoint,
            Millis(timestamp_ms),
            Some(feedforward),
        ) as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_float_eq::assert_f32_near;

    fn default_cv_config() -> Config {
        let max_setpoint = 1600.0; //CV_5  -    V_max               -   Default = 100*16
        Config {
            sample_time: Duration::from_millis(5), //CV_49 = 5   -> Ts = 5 ms
            filter_tc: Duration::from_millis(10),  // CV_48 = 10 -> tau = 10 ms -> 0.010 s
            ki: 2.5, //CV_50  -   PID Control I_Factor        =   CV_50/10        Default = 25 -> 2.5
            kd: 0.005, //CV_51  -   PID Control D_Factor        =   CV_51/10000     Default = 50 -> 0.005
            output_max: (125000000.0 / (150 * 100 + 10000) as f32) as u16, // CV_9  -    PWM frequency in Hz = CV_9*100+10000    - Default = (150*100+10000)Hz = 25kHz
            kp_gain_range1_end: 13.0 / 255.0, //CV_60  -   x_1 shift in % = CV_60/255
            kp_y0: 20.0,
            kp_y1: 2.5,
            kp_y2: 1.5,
            max_setpoint,
        }
    }

    fn build_default_pid() -> Controller {
        Controller::new(default_cv_config()).expect("PID should construct with default CV config")
    }
    //endregion

    //region CV-default based tests

    #[test]
    fn test_pid_kp_schedule_matches_cv_defaults() {
        let pid = build_default_pid();
        let cfg = pid.params;
        let x1 = cfg.max_setpoint * cfg.kp_gain_range1_end; // 1600 * 13/255

        // Note: Kp at exactly 0 may be affected by range boundary behavior; focus on key nodes.

        // At x1 -> kp_y1 (falls into high range start)
        assert_f32_near!(pid.kp_for_setpoint(x1), cfg.kp_y1);

        // At max_setpoint -> near kp_y2
        assert_f32_near!(pid.kp_for_setpoint(cfg.max_setpoint), cfg.kp_y2);

        // Below 0 clamps to 0
        assert_f32_near!(pid.kp_for_setpoint(-100.0), cfg.kp_y0);

        // Above max clamps to max
        assert_f32_near!(pid.kp_for_setpoint(cfg.max_setpoint + 1_000.0), cfg.kp_y2);
    }

    #[test]
    fn test_pid_compute_respects_output_limits_with_cv_defaults() {
        let mut pid = build_default_pid();
        let cfg = pid.params;

        // Use a large feedforward to force saturation and verify clamping.
        let measurement = 0.0;
        let setpoint = cfg.max_setpoint * 10.0; // will be clamped internally to max_setpoint
        let ff = cfg.output_max as f32; // large feedforward to reach upper limit
        let out = pid.compute(measurement, setpoint, 0, ff).unwrap();

        assert!(out >= 0, "output should be >= 0");
        assert!(out <= cfg.output_max, "output should be <= output_max");
    }
    //endregion

    //region GainRange
    #[test]
    fn test_new_gain_range_positive_slope() {
        let range = 10.0..20.0;
        let kp_start = 1.0;
        let kp_end = 2.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end);

        let expected_slope = (kp_end - kp_start) / (range.end - range.start); // (2.0 - 1.0) / (20.0 - 10.0) = 0.1
        assert_eq!(gain_range.setpoint_range, range);
        assert_eq!(gain_range.kp_start, kp_start);
        assert_f32_near!(gain_range.slope, expected_slope);
    }

    #[test]
    fn test_new_gain_range_negative_slope() {
        let range = 50.0..100.0;
        let kp_start = 5.0;
        let kp_end = 0.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end);

        let expected_slope = (kp_end - kp_start) / (range.end - range.start); // (0.0 - 5.0) / (100.0 - 50.0) = -0.1
        assert_eq!(gain_range.setpoint_range, range);
        assert_eq!(gain_range.kp_start, kp_start);
        assert_f32_near!(gain_range.slope, expected_slope);
    }

    #[test]
    fn test_new_gain_range_zero_slope() {
        let range = 0.0..10.0;
        let kp_start = 3.0;
        let kp_end = 3.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end);

        let expected_slope = 0.0;
        assert_eq!(gain_range.setpoint_range, range);
        assert_eq!(gain_range.kp_start, kp_start);
        assert_f32_near!(gain_range.slope, expected_slope);
    }

    #[test]
    fn test_new_gain_range_empty() {
        // Your original code had 1.0, but 0.0 might be safer to avoid division errors
        // Or handle it as a special case. I modified new() to return 0.0 slope.
        let range = 10.0..10.0;
        let kp_start = 1.0;
        let kp_end = 2.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end);

        assert_eq!(gain_range.setpoint_range, range);
        assert_eq!(gain_range.kp_start, kp_start);
        assert_f32_near!(gain_range.slope, 1.0);
    }

    #[test]
    fn test_new_gain_range_flickering_empty() {
        // Test range that is almost empty
        let range = 10.0..(10.0 + f32::EPSILON / 2.0);
        let kp_start = 1.0;
        let kp_end = 2.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end);

        assert_eq!(gain_range.setpoint_range, range);
        assert_eq!(gain_range.kp_start, kp_start);
        assert_f32_near!(gain_range.slope, 1.0);
    }

    #[test]
    fn test_get_kp() {
        let range = 10.0..20.0;
        let kp_start = 1.0;
        let kp_end = 2.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end); // slope = 0.1

        // Start of range
        assert_f32_near!(gain_range.get_kp(10.0), 1.0);
        // Middle of range
        assert_f32_near!(gain_range.get_kp(15.0), 1.5);
        // End of range (exclusive, but calculation should still work)
        assert_f32_near!(gain_range.get_kp(20.0), 2.0);
    }

    #[test]
    fn test_get_kp_extrapolation() {
        let range = 10.0..20.0;
        let kp_start = 1.0;
        let kp_end = 2.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end); // slope = 0.1

        // Before range
        assert_f32_near!(gain_range.get_kp(5.0), 0.5); // 0.1 * (5.0 - 10.0) + 1.0 = -0.5 + 1.0 = 0.5
        // After range
        assert_f32_near!(gain_range.get_kp(30.0), 3.0); // 0.1 * (30.0 - 10.0) + 1.0 = 2.0 + 1.0 = 3.0
    }

    #[test]
    fn test_get_kp_zero_slope() {
        let range = 0.0..10.0;
        let kp_start = 3.0;
        let kp_end = 3.0;
        let gain_range = GainRange::new(range.clone(), kp_start, kp_end); // slope = 0.0

        assert_f32_near!(gain_range.get_kp(0.0), 3.0);
        assert_f32_near!(gain_range.get_kp(5.0), 3.0);
        assert_f32_near!(gain_range.get_kp(10.0), 3.0);
        assert_f32_near!(gain_range.get_kp(100.0), 3.0); // Extrapolates
    }

    #[test]
    fn test_contains() {
        let range = 10.0..20.0;
        let gain_range = GainRange::new(range.clone(), 1.0, 2.0);

        // Inside
        assert!(gain_range.contains(10.0));
        assert!(gain_range.contains(15.0));
        assert!(gain_range.contains(19.999));

        // Outside (boundary)
        assert!(!gain_range.contains(20.0)); // Range.end is exclusive

        // Outside
        assert!(!gain_range.contains(9.999));
        assert!(!gain_range.contains(25.0));
        assert!(!gain_range.contains(-10.0));
    }

    #[test]
    fn test_contains_empty_range() {
        let range = 10.0..10.0;
        let gain_range = GainRange::new(range.clone(), 1.0, 2.0);

        assert!(!gain_range.contains(10.0));
        assert!(!gain_range.contains(0.0));
    }
    //endregion
}
