//! PID controller wrapper built on top of the `discrete_pid` crate.
//!
//! This module adapts the C implementation’s behavior to Rust by using
//! a configurable wrapper that:
//! - mirrors the timing and derivative-on-measurement behavior,
//! - supports gain scheduling for Kp (piecewise linear over the setpoint),
//! - seeds the integral with an initial output (bumpless transfer), so the integral can wind
//!   it back down if it was too large,
//! - applies output limits, and
//! - provides a simple anti-windup policy by holding integration while saturated.

use core::ops::Range;
use core::time::Duration;

use discrete_pid::pid::{PidConfig, PidConfigError, PidController, PidModifiers};
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
/// # Timestamps
///
/// This is a fixed-rate discrete controller. The gains are scaled by the configured
/// [Config::sample_time], and every computed step assumes exactly one sample period has passed:
/// the real interval between calls is never used in the calculation. Timestamps only gate
/// computation: a [Self::compute] whose timestamp is less than one sample period after the
/// previous step returns the previous output without computing.
///
/// Callers must therefore pass the *scheduled* time of each control step (eg: `n *
/// sample_time` from a fixed-rate schedule), not the time at which the step happened to run.
/// Timestamps must never decrease, and consecutive computes must be at least one sample period
/// apart.
///
/// Gain Scheduling:
/// Often it is favorable to have a higher proportional gain KP for slow speeds, achieving
///  better control results.
pub struct Controller {
    pid: PidController<Millis, f32>,
    /// The `discrete_pid` library configuration, kept to rebuild [Self::pid] on
    /// [Self::seed] / [Self::reset].
    inner_config: PidConfig<f32>,
    /// This wrapper's configuration (gain scheduling, setpoint range).
    params: Config,
    // Precomputed gain scheduling parameters
    gain_range_low: GainRange,
    gain_range_high: GainRange,
}

impl Controller {
    /// Creates a new SpeedPid initialized with the provided configuration.
    pub fn new(cfg: Config) -> Result<Self, Error> {
        // Build the underlying library configuration
        let mut inner_config: PidConfig<f32> = PidConfig::default();
        inner_config.set_kp(1.0)?; // Placeholder: Kp is scheduled per call via `PidModifiers::p_scale`
        inner_config.set_ki(cfg.ki)?;
        inner_config.set_kd(cfg.kd)?;
        inner_config.set_filter_tc(cfg.filter_tc.as_secs_f32())?;
        inner_config.set_sample_time(cfg.sample_time)?;
        inner_config.set_output_limits(0.0, cfg.output_max as f32)?;
        inner_config.set_use_derivative_on_measurement(true);
        inner_config.set_use_strict_causal_integrator(true);

        let pid = PidController::new_uninit(inner_config);

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
            inner_config,
            params: cfg,
            gain_range_low: gain_range1,
            gain_range_high: gain_range2,
        })
    }

    /// Reset the controller.
    ///
    /// Useful on direction changes or when stopping. Clears all of the controller state
    /// so the next run doesn't start with a derivative kick from stale state.
    pub fn reset(&mut self) {
        self.pid = PidController::new_uninit(self.inner_config);
    }

    /// Seeds the controller with a bumpless transfer from the startup controller.
    ///
    /// The integral is seeded with `initial_output`, and the derivative state with `measurement`,
    /// so the first output continues from `initial_output`. Unlike a fixed feedforward term, the
    /// integral can then wind down if `initial_output` is too large.
    ///
    /// - `measurement`: latest measured back emf value, corrected for ADC offset.
    /// - `initial_output`: the output to continue from
    /// - `timestamp_ms`: scheduled time of this control step, in milliseconds (see
    ///   [Timestamps](Self#timestamps)). The first [Self::compute] after seeding must be at
    ///   least one sample period later.
    pub fn seed(&mut self, measurement: f32, initial_output: f32, timestamp_ms: u64) {
        self.pid = PidController::new(
            self.inner_config,
            Millis(timestamp_ms),
            measurement,
            initial_output,
        );
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
    /// - `timestamp_ms`: scheduled time of this control step, in milliseconds (see
    ///   [Timestamps](Self#timestamps)). Must be at least one sample period after the previous
    ///   step.
    ///
    /// Call [Self::seed] first to seed the controller; otherwise it starts from zero output.
    pub fn compute(&mut self, measurement: f32, setpoint: f32, timestamp_ms: u64) -> u16 {
        // An earlier timestamp would be skipped by the library (or, if it went backwards, panic or
        // wrap in its time subtraction). Either means the caller isn't using a fixed-rate schedule.
        debug_assert!(
            self.pid.last_time().is_none_or(|last| {
                // Same comparison as the library's gate, so this holds for any sample time.
                // checked_sub catches a timestamp going backwards without panicking here.
                timestamp_ms
                    .checked_sub(last.0)
                    .is_some_and(|dt| Duration::from_millis(dt) >= self.inner_config.sample_time())
            }),
            "PID step at {}ms is less than one sample period after the previous step at {:?}ms",
            timestamp_ms,
            self.pid.last_time().map(|t| t.0),
        );

        let setpoint = setpoint.clamp(0.0, self.params.max_setpoint);

        // Anti-windup: hold integration that would push further into saturation, based on the
        // previous output.
        let prev_output = self.pid.output();
        let mods = PidModifiers {
            // Dynamic Kp scheduling based on setpoint (the base kp is 1)
            p_scale: self.kp_for_setpoint(setpoint),
            allow_integral_increase: prev_output < self.inner_config.output_max(),
            allow_integral_decrease: prev_output > self.inner_config.output_min(),
            ..PidModifiers::default()
        };

        // Compute control output. The output is clamped to [0, output_max], so the cast is lossless
        // apart from truncating the fraction.
        self.pid
            .compute_with_modifiers(measurement, setpoint, Millis(timestamp_ms), None, &mods)
            as u16
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

        // Use a large initial output to force saturation and verify clamping.
        let measurement = 0.0;
        let setpoint = cfg.max_setpoint * 10.0; // will be clamped internally to max_setpoint
        pid.seed(measurement, cfg.output_max as f32 * 2.0, 0);
        let out = pid.compute(measurement, setpoint, 5);

        assert!(out <= cfg.output_max, "output should be <= output_max");
    }
    //endregion

    //region seed / anti-windup

    /// Sample time of [default_cv_config], in ms.
    const TS: u64 = 5;

    /// Integral increment for one step at `error`, with the default CV config.
    fn integral_step(error: f32) -> f32 {
        let cfg = default_cv_config();
        cfg.ki * cfg.sample_time.as_secs_f32() * error
    }

    #[test]
    fn test_seed_is_bumpless() {
        let mut pid = build_default_pid();

        // With zero error, the first output continues from the seeded value (no P, I or D kick).
        pid.seed(100.0, 3000.0, 0);
        let out = pid.compute(100.0, 100.0, TS);

        assert_eq!(out, 3000);
    }

    #[test]
    fn test_seed_replaces_previous_state() {
        let mut pid = build_default_pid();

        // Run fast for a while.
        pid.seed(400.0, 4000.0, 0);
        for i in 1..=100 {
            pid.compute(400.0, 400.0, i * TS);
        }

        // Seed again at a low speed: no derivative kick from the previous 400 measurement, and
        // no leftover integral.
        let t = 1000;
        pid.seed(10.0, 1000.0, t);
        let out = pid.compute(10.0, 10.0, t + TS);

        assert_eq!(out, 1000);
    }

    #[test]
    fn test_reset_clears_derivative_and_integral_state() {
        let mut pid = build_default_pid();

        // Run fast for a while.
        pid.seed(400.0, 4000.0, 0);
        for i in 1..=100 {
            pid.compute(400.0, 400.0, i * TS);
        }
        pid.reset();

        // After a reset (without seeding), the controller starts from zero: no integral left over
        // and no derivative kick from the previous 400 measurement. Only clearing the integral
        // would leave the old measurement in the derivative term and give a non-zero output here.
        let out = pid.compute(10.0, 10.0, 1000);

        assert_eq!(out, 0);
    }

    #[test]
    fn test_integral_winds_down_an_oversized_initial_output() {
        let mut pid = build_default_pid();

        // Startup handed over with an initial output that is far too large for the setpoint: the
        // motor runs faster than the target.
        let setpoint = 12.0;
        let measurement = 40.0;
        let steps = 2000;
        pid.seed(measurement, 4000.0, 0);

        let first = pid.compute(measurement, setpoint, TS);
        let mut out = first;
        for i in 2..=steps {
            out = pid.compute(measurement, setpoint, i * TS);
        }

        // The integral must be able to go below the initial output (a fixed feedforward term
        // with the integral clamped at 0 would never drop below `first`), and at the rate set by
        // ki: P and D are constant, so the output falls by one integral step per sample.
        let expected = -integral_step(setpoint - measurement) * (steps - 1) as f32;
        let unwound = f32::from(first) - f32::from(out);
        assert!(
            (unwound - expected).abs() <= 2.0,
            "output unwound by {unwound}, expected {expected}"
        );
    }

    #[test]
    fn test_no_windup_while_saturated_high() {
        let mut pid = build_default_pid();
        let cfg = pid.params;
        let seeded_output = 3000.0;

        // Large positive error saturates the output at max.
        pid.seed(0.0, seeded_output, 0);
        for i in 1..=500 {
            let out = pid.compute(0.0, cfg.max_setpoint, i * TS);
            assert_eq!(out, cfg.output_max);
        }

        // Error goes to zero with the same measurement (so no derivative term): the output is just
        // the integral. Only the first step, before saturation was detected, may have integrated;
        // without anti-windup the integral would have wound up to output_max.
        let out = pid.compute(0.0, 0.0, 501 * TS);
        let max_integral = seeded_output + integral_step(cfg.max_setpoint);
        assert!(
            f32::from(out) <= max_integral,
            "integral wound up to {out} while saturated (limit {max_integral})"
        );
    }

    #[test]
    fn test_no_windup_while_saturated_low() {
        let mut pid = build_default_pid();
        let cfg = pid.params;
        let seeded_output = 1000.0;

        // Large negative error (the motor is much faster than the target) saturates the output at 0.
        pid.seed(cfg.max_setpoint, seeded_output, 0);
        for i in 1..=500 {
            let out = pid.compute(cfg.max_setpoint, 0.0, i * TS);
            assert_eq!(out, 0);
        }

        // Error goes to zero with the same measurement: the output is just the integral. Only the
        // first step may have integrated; without anti-windup the integral would have wound down
        // to 0.
        let out = pid.compute(cfg.max_setpoint, cfg.max_setpoint, 501 * TS);
        let min_integral = seeded_output + integral_step(-cfg.max_setpoint);
        assert!(
            f32::from(out) >= min_integral - 1.0,
            "integral wound down to {out} while saturated (limit {min_integral})"
        );
    }

    #[test]
    fn test_steps_one_sample_period_apart_are_never_skipped() {
        let mut pid = build_default_pid();
        let cfg = pid.params;

        // A constant error large enough that each step's integral increment shows in the
        // (truncated) u16 output, without saturating.
        let measurement = cfg.max_setpoint - 600.0;
        let setpoint = cfg.max_setpoint;
        assert!(integral_step(setpoint - measurement) >= 2.0);

        pid.seed(measurement, 1000.0, 0);
        let mut prev = pid.compute(measurement, setpoint, TS);
        for i in 2..=50 {
            let out = pid.compute(measurement, setpoint, i * TS);
            assert!(out < cfg.output_max, "test saturated at step {i}");
            // a skipped step would return the previous output unchanged
            assert!(out > prev, "step {i} was skipped: output {out} == {prev}");
            prev = out;
        }
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "less than one sample period")]
    fn test_step_less_than_one_sample_period_apart_panics_in_debug() {
        let mut pid = build_default_pid();

        pid.seed(100.0, 1000.0, 0);
        pid.compute(100.0, 100.0, TS);
        // A caller passing the time steps actually ran (with jitter) rather than their scheduled
        // time could do this; the library would silently skip the step.
        pid.compute(100.0, 100.0, 2 * TS - 1);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "less than one sample period")]
    fn test_first_step_after_seed_must_be_one_sample_period_later() {
        let mut pid = build_default_pid();

        pid.seed(100.0, 1000.0, 10);
        pid.compute(100.0, 100.0, 10);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "less than one sample period")]
    fn test_step_going_backwards_panics_in_debug() {
        let mut pid = build_default_pid();

        pid.seed(100.0, 1000.0, 10 * TS);
        pid.compute(100.0, 100.0, 11 * TS);
        // caught by the assertion rather than underflowing in the library's time subtraction
        pid.compute(100.0, 100.0, 5 * TS);
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
