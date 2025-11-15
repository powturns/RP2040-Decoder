
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

use discrete_pid::{pid, time};
use discrete_pid::pid::{IntegratorActivity, PidConfig, PidConfigError, PidController};
use discrete_pid::time::Millis;

#[derive(Eq, PartialEq)]
pub enum Error {
    ConfigError(PidConfigError)
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
pub struct SpeedPidConfig {
    /// Sampling time in milliseconds (matches the C sampling period CV).
    pub sample_time_ms: u64,

    /// Low-pass filter time constant (seconds) used on the derivative term (tau in C).
    pub filter_tau_s: f32,

    /// Time-invariant integral gain (ki). The library scales this by the sample time.
    ///
    /// FIXME
    /// CV 50 / 10
    pub ki: f32,

    /// Time-invariant derivative gain (kd). The library scales this by the sample time.
    ///
    /// FIXME
    /// CV 51 / 10_000
    pub kd: f32,

    /// Minimum controller output. Usually 0.0 for a PWM duty lower bound.
    pub output_min: f32,

    /// Maximum controller output (PWM top).
    ///
    /// FIXME
    /// (float) (_125M / (CV9 * 100 + 10000));
    pub output_max: f32,

    /// Use derivative on measurement instead of error (matches C implementation).
    pub derivative_on_measurement: bool,

    /// Use strict causal integrator (updates I after output), closer to C ordering.
    pub strict_causal_integrator: bool,

    /// ADC offset to subtract from the measurement before control.
    ///
    /// CV 172
    pub adc_offset: f32,

    /// Optional feed-forward scaling factor (k_ff in C). If you compute feedforward
    /// externally, you can pass None at runtime to override this per call.
    pub kff: f32,

    /// Motion detection threshold for deciding when motion exists (units of measurement).
    /// Not directly used by the core PID, but provided for symmetry with C if needed by caller.
    pub motion_threshold: f32,

    /// Gain scheduling parameters for Kp based on the setpoint.
    /// Defines the position of the end of the first gain range based on a fraction of the [max_setpoint].
    pub kp_gain_range1_end: f32,

    /// Kp level at x0
    pub kp_y0: f32,

    /// Kp level at x1
    pub kp_y1: f32,

    /// Kp level at x2
    pub kp_y2: f32,

    /// Maximal value of the setpoint range (e.g., last speed table entry).
    pub max_setpoint: f32,
}

impl Default for SpeedPidConfig {
    fn default() -> Self {
        Self {
            sample_time_ms: 10,
            filter_tau_s: 0.01,
            ki: 0.01,
            kd: 0.0,
            output_min: 0.0,
            output_max: f32::INFINITY,
            derivative_on_measurement: true,
            strict_causal_integrator: true,
            adc_offset: 0.0,
            kff: 0.0,
            motion_threshold: 7.5,
            kp_gain_range1_end: 0.5,
            kp_y0: 1.0,
            kp_y1: 1.0,
            kp_y2: 1.0,
            max_setpoint: 1.0,
        }
    }
}

struct GainRange {
    setpoint_range: Range<f32>,
    kp_start: f32,
    slope: f32,
}

impl GainRange {
    fn new(
        setpoint_range: Range<f32>,
        kp_start: f32,
        kp_end: f32,
    ) -> Self {
        let slope = if setpoint_range.is_empty() {
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
}

/// PID controller wrapper with gain scheduling and simple anti-windup.
///
/// Gain Scheduling:
/// Often it is favorable to have a higher proportional gain KP for slow speeds, achieving
///  better control results.
pub struct SpeedPid {
    pid: PidController<Millis, f32>,
    params: SpeedPidConfig,
    // Precomputed gain scheduling parameters
    gain_range_low: GainRange,
    gain_range_high: GainRange,
}

impl SpeedPid {
    /// Creates a new SpeedPid initialized with the provided configuration.
    pub fn new(cfg: SpeedPidConfig) -> Result<Self, Error> {
        // Build the underlying library configuration
        let mut lib_cfg: PidConfig<f32> = pid::PidConfig::default();
        lib_cfg.set_kp(1.0)?; // placeholder; will be updated dynamically per setpoint
        lib_cfg.set_ki(cfg.ki)?;
        lib_cfg.set_kd(cfg.kd)?;
        lib_cfg.set_filter_tc(cfg.filter_tau_s)?;
        lib_cfg.set_sample_time(Duration::from_millis(cfg.sample_time_ms))?;
        lib_cfg.set_output_limits(cfg.output_min, cfg.output_max)?;
        lib_cfg.set_use_derivative_on_measurement(cfg.derivative_on_measurement);
        lib_cfg.set_use_strict_causal_integrator(cfg.strict_causal_integrator);

        let pid = PidController::new_uninit(lib_cfg);

        // Gain scheduling precompute
        let kp_x1 = cfg.max_setpoint * cfg.kp_gain_range1_end;

        let gain_range1 = GainRange::new(
            Range { start: 0.0, end: kp_x1 },
            cfg.kp_y0,
            cfg.kp_y1,
        );

        let gain_range2 = GainRange::new(
            Range { start: kp_x1, end: cfg.max_setpoint },
            cfg.kp_y1,
            cfg.kp_y2,
        );

        Ok(Self {
            pid,
            params: cfg,
            gain_range_low: gain_range1,
            gain_range_high: gain_range2
        })
    }

    /// Resets the integral accumulation. Useful on direction changes or when stopping.
    pub fn reset_integral(&mut self) {
        self.pid.reset_integral();
    }

    /// Computes the scheduled Kp based on the provided setpoint.
    ///
    /// Often it is favorable to have a higher proportional gain KP for slow speeds, achieving
    /// better control results.
    /// 
    fn kp_for_setpoint(&self, sp: f32) -> f32 {
        if sp < self.kp_x1 {
            self.kp_m1 * sp + self.params.kp_y0
        } else {
            self.kp_m2 * (sp - self.kp_x1) + self.params.kp_y1
        }
    }

    /// Apply a simple anti-windup policy by holding integrator at saturation when error pushes
    /// further into saturation. Otherwise keep integration active.
    fn update_integrator_activity(&mut self, error: f32) {
        let cfg = self.pid.config();
        let out = self.pid.output();
        let eps = (cfg.output_max() - cfg.output_min()) * 1e-6;
        let at_high = (cfg.output_max() - out) <= eps;
        let at_low = (out - cfg.output_min()) <= eps;

        let activity = if (at_high && error > 0.0) || (at_low && error < 0.0) {
            IntegratorActivity::HoldIntegration
        } else {
            IntegratorActivity::Active
        };
        self.pid.set_integrator_activity(activity);
    }

    /// Compute a control output.
    ///
    /// - `measurement`: latest measured value (same units as the setpoint), prior to offset removal
    /// - `setpoint`: desired target value
    /// - `timestamp_ms`: current time in milliseconds
    /// - `feedforward`: Optional feedforward term. If None, uses kff*setpoint as a simple default.
    pub fn compute(
        &mut self,
        measurement: f32,
        setpoint: f32,
        timestamp_ms: u64,
        feedforward: Option<f32>,
    ) -> f32 {
        // Correct measurement by ADC offset
        let input = measurement - self.params.adc_offset;

        // Dynamic Kp scheduling based on setpoint
        let kp = self.kp_for_setpoint(setpoint);
        // Update Kp; ignore error since kp>0 by construction in normal operation
        let _ = self.pid.config_mut().set_kp(kp.max(core::f32::EPSILON));

        // Anti-windup policy based on current saturation and error direction
        let error = setpoint - input;
        self.update_integrator_activity(error);

        // Compute feedforward
        let ff = match feedforward {
            Some(v) => Some(v),
            None => Some(self.params.kff * setpoint),
        };

        // Compute control output
        let out = self
            .pid
            .compute(input, setpoint, time::Millis(timestamp_ms), ff);
        out
    }

    /// Returns the most recent output value from the underlying PID controller.
    pub fn last_output(&self) -> f32 {
        self.pid.output()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gain_range() {
        let default_low = GainRange::new(Range { start: 0.0, end: 1.0 }, 1.0, 2.0);
        let default_high = GainRange::new(Range { start: 1.0, end: 2.0 }, 2.0, 3.0);


    }
}