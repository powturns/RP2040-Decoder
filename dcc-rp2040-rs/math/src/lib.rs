#![no_std]

use libm::{round, sqrtf};

/// Calculates the mean of the [data] filtering out any elements that are
/// more than [std_deviations] away from the mean.
pub fn filtered_mean(data: &[u16], std_deviations: u16) -> Option<u16> {
    if data.is_empty() {
        return None;
    }

    let mean: u16 = (data.iter().map(|&e| e as u64).sum::<u64>() / data.len() as u64) as u16;

    if data.len() == 1 {
        return Some(mean);
    }

    // calculate standard deviation
    let std_dev: f32 = {
        let var_sum = data
            .iter()
            .map(|&e| (e as i64 - mean as i64).pow(2) as u64)
            .sum::<u64>();
        let variance = (var_sum as f64 / (data.len() as f64 - 1.0)) as f32;

        sqrtf(variance)
    };

    let threshold = std_deviations as f64 * std_dev as f64;

    let (sum, cnt) = data.iter().fold((0_u64, 0_usize), |(sum, cnt), &e| {
        if e.abs_diff(mean) as f64 <= threshold {
            (sum + e as u64, cnt + 1)
        } else {
            (sum, cnt)
        }
    });

    if cnt == 0 {
        None
    } else {
        Some(round(sum as f64 / cnt as f64) as u16)
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
}
