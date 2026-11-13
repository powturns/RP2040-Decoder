//! BEMF calibration run.
//!
//! The motor is driven open-loop: the output is ramped up to 100%, held while the motor settles,
//! then the back-EMF is sampled. The measured full-output BEMF is used to derive CV5 (vhigh) and
//! CV6 (vmid) for the speed table.
//!
//! This module is hardware-free; [Calibrator::step] must be called once per control tick.

use crate::speed::CONTROLLER_HANDOVER;
use embassy_time::Duration;

/// Upper bound on stored samples.
pub const MAX_SAMPLES: usize = 128;

/// Status value for a successful run.
pub const STATUS_OK: u8 = 2;

const RAMP_TIME: Duration = Duration::from_millis(600);

/// The maximum time to wait for a stable sample.
const MAX_SAMPLE_TIME: Duration = Duration::from_millis(3000);

/// Maximum relative difference between the first and second half of the samples.
const STABILITY_LIMIT: f32 = 0.03;

/// Fraction of samples dropped from each end when computing the trimmed mean.
const TRIM_FRACTION: f32 = 0.1;

/// Below this v_high value the motor barely turned.
const MIN_V_HIGH: f32 = 4.0;

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct Config {
    /// PWM level that is 100% output.
    pub output_max: u16,
    /// The control tick period. `step` must be called once per tick.
    pub sample_time: Duration,
    /// ADC offset subtracted from every raw measurement.
    pub adc_offset: f32,
}

/// CV values derived from a calibration run.
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct Calibration {
    pub v_high: u8,
    pub v_mid: u8,
}

#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Failure {
    /// The run was aborted.
    Aborted,
    /// The motor did not reach the handover BEMF at full output.
    NoMotion,
    /// The BEMF was not steady while sampling.
    Unstable,
    /// The result is outside the usable range.
    OutOfRange,
}

impl Failure {
    /// Value written to the calibration status CV (CV176).
    pub fn status(self) -> u8 {
        match self {
            Failure::Aborted => 3,
            Failure::NoMotion => 4,
            Failure::Unstable => 5,
            Failure::OutOfRange => 6,
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Step {
    /// Apply this output level and call `step` again next tick.
    Output(u16),
    /// The run is over; stop the motor. Ok carries emf_max: the offset-corrected
    /// BEMF at full output, in ADC counts.
    Done(Result<u16, Failure>),
}

#[derive(Copy, Clone)]
enum Phase {
    Ramp,
    Sample,
    Finished(Result<u16, Failure>),
}

pub struct Calibrator {
    output_max: u16,
    adc_offset: f32,
    ramp_ticks: u32,
    max_sample_ticks: u32,
    phase: Phase,
    /// Ticks elapsed in the current phase.
    tick: u32,
    samples: [f32; MAX_SAMPLES],
    count: usize,
}

/// Converts a `duration` into the number of ticks, when
/// [Calibrator::step] is called every `sample_time`.
fn ticks(duration: Duration, sample_time: Duration) -> u32 {
    let sample_us = sample_time.as_micros();
    if sample_us == 0 {
        return 1;
    }
    let us = duration.as_micros();
    u32::try_from(us.div_ceil(sample_us))
        .unwrap_or(u32::MAX)
        .max(1)
}

impl Calibrator {
    pub fn new(config: Config) -> Self {
        Self {
            output_max: config.output_max,
            adc_offset: config.adc_offset,
            ramp_ticks: ticks(RAMP_TIME, config.sample_time),
            max_sample_ticks: u32::max(
                ticks(MAX_SAMPLE_TIME, config.sample_time),
                MAX_SAMPLES as u32 // always sample at least once
            ),
            phase: Phase::Ramp,
            tick: 0,
            samples: [0.0; MAX_SAMPLES],
            count: 0,
        }
    }

    /// Advances the run by one tick. `measurement` is the raw (not offset-corrected)
    /// BEMF, taken with the output from the previous step applied.
    pub fn step(&mut self, measurement: f32) -> Step {
        let emf = (measurement - self.adc_offset).max(0.0);
        match self.phase {
            Phase::Ramp => {
                // Gradually ramp up to 100% throttle
                self.tick += 1;
                if self.tick >= self.ramp_ticks {
                    if emf < CONTROLLER_HANDOVER {
                        let result = Err(Failure::NoMotion);
                        self.phase = Phase::Finished(result);
                        return Step::Done(result);
                    }

                    // Ramp phase complete and motor is turning
                    self.phase = Phase::Sample;
                    self.tick = 0;
                    return Step::Output(self.output_max);
                }

                // reach 100% throttle with 10% of the ramp ticks remaining so we have a little settle time
                let out = u32::min(
                    self.output_max as u32 * self.tick / (self.ramp_ticks as f32 * 0.9) as u32,
                    self.output_max as u32,
                );
                Step::Output(out as u16)
            }
            Phase::Sample => {
                // In the sample phase keep sampling until the reading is stable OR
                // we exhaust the max_sample_ticks

                if self.tick > self.max_sample_ticks {
                    // if we've exhausted the sample ticks it means we never got a stable reading
                    let result = Err(Failure::Unstable);
                    self.phase = Phase::Finished(result);
                    return Step::Done(result)
                }

                self.samples[self.count] = emf;
                self.count += 1;
                self.tick += 1;

                // if we've captured enough samples, evaluate the outcome and finish if its
                // stable. Otherwise keep iterating.
                if self.count >= self.samples.len() {
                    let result = self.evaluate();
                    if result.is_ok() {
                        self.phase = Phase::Finished(result);
                        return Step::Done(result);
                    }

                    // reset the count to preform a new fresh sample
                    self.count = 0;
                }
                Step::Output(self.output_max)
            }

            // The caller called step after Done was returned the first
            // time, this should never happen, but return the same result again.
            Phase::Finished(result) => Step::Done(result),
        }
    }

    fn evaluate(&mut self) -> Result<u16, Failure> {
        let samples = &mut self.samples[..self.count];
        let half = samples.len() / 2;

        // check the quality of the data. Calculate the mean of both halves of the samples
        // and if it is too different, return an error.
        let (first, second) = samples.split_at_mut(half);
        let a = trimmed_mean(first);
        let b = trimmed_mean(second);
        if (b - a).abs() > STABILITY_LIMIT * a.max(1.0) {
            warn!(
                "calibration unstable: first half={} second half={} samples={}",
                a,
                b,
                self.count
            );
            return Err(Failure::Unstable);
        }

        // calculate the mean emf value at max power
        let emf_max = trimmed_mean(samples);

        // add 0.5 to round up to the nearest integer (any fractional portion is truncated)
        Ok((emf_max + 0.5) as u16)
    }
}

/// Sorts `samples` in place, drops 10% from each end and returns the mean of the rest.
/// Returns 0 for an empty slice.
fn trimmed_mean(samples: &mut [f32]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    samples.sort_unstable_by(f32::total_cmp);
    let trim = (samples.len() as f32 * TRIM_FRACTION) as usize;
    // trim <= len / 10, so the kept slice is never empty.
    let kept = &samples[trim..samples.len() - trim];
    let sum: f32 = kept.iter().sum();
    sum / kept.len() as f32
}

/// Derives CV5 / CV6 from the measured full-output BEMF.
/// `headroom` is CV179 / 255. `v_high_old` / `v_mid_old` are CV5 / CV6 before calibration.
pub fn compute_cvs(
    emf_max: u16,
    headroom: f32,
    v_high_old: u8,
    v_mid_old: u8,
) -> Result<Calibration, Failure> {
    let v_high_f = emf_max as f32 * headroom / 16.0;
    if v_high_f < MIN_V_HIGH {
        return Err(Failure::OutOfRange);
    }
    let v_high = (v_high_f + 0.5) as u8;

    let v_mid = if v_high_old == 0 {
        v_high / 2
    } else {
        let m = v_mid_old as f32 * v_high as f32 / v_high_old as f32;
        ((m + 0.5) as u8).min(v_high)
    };

    Ok(Calibration { v_high, v_mid })
}

#[cfg(test)]
mod tests {
    use super::*;

    const TS: Duration = Duration::from_millis(5);
    const MAX: u16 = 1000;

    fn cal(sample_time: Duration, offset: f32) -> Calibrator {
        Calibrator::new(Config {
            output_max: MAX,
            sample_time,
            adc_offset: offset,
        })
    }

    /// Runs until Done, returning the result and the number of steps taken.
    fn run(c: &mut Calibrator, mut meas: impl FnMut(u32) -> f32) -> (Result<u16, Failure>, u32) {
        let mut n = 0;
        loop {
            let m = meas(n);
            n += 1;
            if let Step::Done(r) = c.step(m) {
                return (r, n);
            }
            assert!(n < 10_000);
        }
    }

    #[test]
    fn ramp_is_linear_then_holds() {
        let mut c = cal(TS, 0.0);
        for k in 1..120u32 {
            assert_eq!(c.step(100.0), Step::Output((MAX as u32 * k / 120) as u16));
        }
        // last ramp tick hands over to full output
        assert_eq!(c.step(100.0), Step::Output(MAX));
        for _ in 0..99 {
            assert_eq!(c.step(100.0), Step::Output(MAX));
        }
    }

    #[test]
    fn no_motion() {
        let mut c = cal(TS, 10.0);
        let (r, n) = run(&mut c, |_| 17.0); // 7.0 after offset
        assert_eq!(r, Err(Failure::NoMotion));
        assert_eq!(n, 120);
    }

    #[test]
    fn constant_measurement() {
        let mut c = cal(TS, 10.0);
        let (r, n) = run(&mut c, |_| 110.0);
        assert_eq!(r, Ok(100));
        assert_eq!(n, 120 + 300 + 100);
    }

    fn two_halves(second: f32) -> Result<u16, Failure> {
        let mut c = cal(TS, 0.0);
        // samples start after 420 ticks, 100 of them
        run(&mut c, |i| if i >= 470 { second } else { 100.0 }).0
    }

    #[test]
    fn stability() {
        assert_eq!(two_halves(105.0), Err(Failure::Unstable));
        assert!(two_halves(102.0).is_ok());
    }

    #[test]
    fn outliers_trimmed() {
        let mut c = cal(TS, 0.0);
        let (r, _) = run(&mut c, |i| match i {
            430 | 431 | 490 => 0.0,
            440 | 480 | 500 => 5000.0,
            _ => 200.0,
        });
        assert_eq!(r, Ok(200));
    }

    #[test]
    fn compute_cvs_values() {
        let h = 235.0 / 255.0;
        let c = compute_cvs(550, h, 100, 63).unwrap();
        assert_eq!(c.v_high, 32);
        assert_eq!(c.v_mid, 20);

        // v_mid never exceeds v_high
        let c = compute_cvs(550, h, 10, 200).unwrap();
        assert_eq!(c.v_mid, c.v_high);

        // no previous v_high: half
        let c = compute_cvs(550, h, 0, 63).unwrap();
        assert_eq!(c.v_mid, 16);

        // saturates
        let c = compute_cvs(u16::MAX, 1.0, 100, 50).unwrap();
        assert_eq!(c.v_high, 255);

        assert_eq!(compute_cvs(10, h, 100, 50), Err(Failure::OutOfRange));
    }

    #[test]
    fn one_ms_stops_at_max_samples() {
        let mut c = cal(Duration::from_millis(1), 0.0);
        let (r, n) = run(&mut c, |_| 100.0);
        assert_eq!(r, Ok(100));
        assert_eq!(n, 600 + 1500 + MAX_SAMPLES as u32);
    }

    #[test]
    fn same_result_after_done() {
        let mut c = cal(TS, 0.0);
        let (r, _) = run(&mut c, |_| 100.0);
        assert_eq!(c.step(100.0), Step::Done(r));
        assert_eq!(c.step(100.0), Step::Done(r));

        let mut c = cal(TS, 0.0);
        let (r, _) = run(&mut c, |_| 0.0);
        assert_eq!(c.step(100.0), Step::Done(r));
    }

    #[test]
    fn failure_status() {
        assert_eq!(Failure::Aborted.status(), 3);
        assert_eq!(Failure::NoMotion.status(), 4);
        assert_eq!(Failure::Unstable.status(), 5);
        assert_eq!(Failure::OutOfRange.status(), 6);
        assert_eq!(STATUS_OK, 2);
    }
}
