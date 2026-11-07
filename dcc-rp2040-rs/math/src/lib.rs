#![no_std]

use core::ops::Add;

/// Longest input whose `Σe` is guaranteed to fit in a `u32` accumulator.
const MAX_LEN_U32_SUM: usize = (u32::MAX / u16::MAX as u32) as usize;

/// Running-sum accumulator type for the per-element loops.
///
/// The M0+ has no 64-bit registers: a `u64` total occupies two registers and
/// needs an `adds`/`adcs` pair, and the extra pressure spills loop state to
/// the stack. A `u32` total is used whenever the input is short enough.
trait Acc: Copy + Default + Add<Output = Self> + From<u16> + Into<u64> {}
impl Acc for u32 {}
impl Acc for u64 {}

/// Pass 1: returns `(Σe, Σe²)`.
#[inline(always)]
fn sum_and_sum_sq<A: Acc>(data: &[u16]) -> (u64, u64) {
    let (sum, sum_sq) = data.iter().fold((A::default(), 0_u64), |(s, sq), &e| {
        let e32 = e as u32;
        // e² <= (2^16-1)² fits in u32, so this is a single native multiply.
        (s + A::from(e), sq.wrapping_add((e32 * e32) as u64))
    });
    (sum.into(), sum_sq)
}

/// Pass 2: returns `(Σe, count)` over elements within `max_dev` of `mean`.
#[inline(always)]
fn inlier_sum<A: Acc>(data: &[u16], mean: u16, max_dev: u16) -> (u64, u32) {
    let (sum, cnt) = data.iter().fold((A::default(), 0_u32), |(s, c), &e| {
        if e.abs_diff(mean) <= max_dev {
            (s + A::from(e), c + 1)
        } else {
            (s, c)
        }
    });
    (sum.into(), cnt)
}

/// Calculates the mean of the [data] filtering out any elements that are
/// more than [std_deviations] away from the mean.
///
/// Implemented entirely in integer arithmetic: the RP2040 (Cortex-M0+) has no
/// FPU, so every `f32`/`f64` operation is a costly soft-float library call.
///
/// * Pass 1 accumulates `Σe` and `Σe²` together; the squares are 32x32->32
///   multiplies (single cycle on the M0+). The sum of squared deviations from
///   the (truncated integer) mean is then derived exactly via
///   `Σ(e-m)² = Σe² - 2mΣe + nm²`, avoiding a separate variance pass.
/// * The filter test `|e-m| <= k·sqrt(var_sum / (n-1))` is squared and solved
///   once for the largest admissible integer deviation, so pass 2 is just a
///   `u16` compare and add per element.
pub fn filtered_mean(data: &[u16], std_deviations: u16) -> Option<u16> {
    let n = data.len();
    if n == 0 {
        return None;
    }

    let small = n <= MAX_LEN_U32_SUM;

    // Pass 1: Σe and Σe².
    let (sum, sum_sq) = if small {
        sum_and_sum_sq::<u32>(data)
    } else {
        sum_and_sum_sq::<u64>(data)
    };

    let n64 = n as u64;
    let mean = (sum / n64) as u16;
    if n == 1 {
        return Some(mean);
    }

    // Σ(e-m)² = Σe² - 2mΣe + nm². Intermediates may wrap, but the true result
    // is <= n·(2^16)² and fits in u64, so modular arithmetic yields it exactly.
    let m = mean as u64;
    let var_sum = sum_sq
        .wrapping_sub(2_u64.wrapping_mul(m).wrapping_mul(sum))
        .wrapping_add(n64.wrapping_mul(m * m));

    // Keep e iff  d² <= k²·var_sum / (n-1)   (d = |e-m|, integer)
    //        iff  d  <= isqrt(floor(k²·var_sum / (n-1)))
    // Saturation is safe: d²·(n-1) can never reach u64::MAX.
    let k = std_deviations as u64;
    let bound = (k * k).saturating_mul(var_sum) / (n64 - 1);
    let max_dev = bound.isqrt().min(u16::MAX as u64) as u16;

    // Pass 2: sum of the inliers.
    let (sum, cnt) = if small {
        inlier_sum::<u32>(data, mean, max_dev)
    } else {
        inlier_sum::<u64>(data, mean, max_dev)
    };

    if cnt == 0 {
        None
    } else {
        // Round half away from zero (all values are non-negative).
        let cnt = cnt as u64;
        Some(((sum + cnt / 2) / cnt) as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_input() {
        let data = [];
        assert_eq!(filtered_mean(&data, 2), None);
    }

    #[test]
    fn test_single_element() {
        let data = [42];
        // A single element is its own mean, variance is mathematically undefined
        // or treated as 0, logic should simply return the element.
        assert_eq!(filtered_mean(&data, 2), Some(42));
    }

    #[test]
    fn test_identical_elements() {
        let data = [10, 10, 10, 10];
        // Mean 10, StdDev 0. All elements included.
        assert_eq!(filtered_mean(&data, 1), Some(10));
    }

    #[test]
    fn test_basic_outlier_removal() {
        let data = [10, 12, 11, 10, 100];

        let result = filtered_mean(&data, 1);

        // We expect result to be close to 11 (average of 10, 12, 11, 10)
        assert_eq!(result, Some(11));
    }

    #[test]
    fn test_no_outliers_removed_with_high_tolerance() {
        let data = [10, 12, 11, 10, 50];
        // If we allow 3 standard deviations, 50 should likely stay included
        // because the std_dev itself will be large due to the 50.
        let result = filtered_mean(&data, 3);

        // Sum: 93, Count: 5, Mean: 18.6
        assert_eq!(result, Some(19)); // 18.6 rounds to 19
    }

    #[test]
    fn test_small_variance_precision() {
        // In the original integer code, variance of 0.5 became 0.
        // Here: Mean is 10.5.
        // Variance sum: (0.5^2 + 0.5^2) = 0.5.
        // Variance = 0.5 / 1 = 0.5. StdDev = 0.707.
        // Threshold (2 sigmas) = 1.414.
        // Diff is 0.5. 0.5 <= 1.414. Both kept.
        let data = [10, 11];
        let result = filtered_mean(&data, 2);

        // Average of 10 and 11 is 10.5 -> rounds to 11
        assert_eq!(result, Some(11));
    }

    #[test]
    fn test_two_distinct_clusters() {
        // Cluster A: 10, 10, 10
        // Cluster B: 30, 30, 30
        // Mean: 20. Std Dev: 10.
        // 1 Sigma range: [10, 30].
        // All should be kept.
        let data = [10, 10, 10, 30, 30, 30];
        assert_eq!(filtered_mean(&data, 1), Some(20));
    }

    #[test]
    fn test_aggressive_filtering() {
        // Mean: 50
        // If we set std_deviations to 0, we only keep elements exactly equal to the mean.
        let data = [40, 50, 60];
        // Mean is 50.
        // 40 diff is 10. 60 diff is 10. 50 diff is 0.
        // Threshold = 0 * std_dev = 0.
        // Only 50 should remain.
        assert_eq!(filtered_mean(&data, 0), Some(50));
    }

    #[test]
    fn test_u32_accumulator_limit() {
        // Largest input using the u32 path: Σe == u32::MAX exactly.
        let data = [u16::MAX; MAX_LEN_U32_SUM];
        assert_eq!(filtered_mean(&data, 2), Some(u16::MAX));
    }

    #[test]
    fn test_u64_accumulator_fallback() {
        // One past the limit: Σe overflows u32, so the u64 path must be taken.
        let data = [u16::MAX; MAX_LEN_U32_SUM + 1];
        assert_eq!(filtered_mean(&data, 2), Some(u16::MAX));
    }
}
