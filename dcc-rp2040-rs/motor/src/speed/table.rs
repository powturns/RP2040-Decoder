const SPEED_TABLE_LEN: usize = 127;
pub type SpeedTable = [u16; SPEED_TABLE_LEN];

#[derive(Eq, PartialEq)]
#[cfg_attr(test, derive(Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Used to define the bemf level used as the start voltage on the motor.
    pub v_start: u8,

    /// Vmid specifies the bemf drive level at the middle speed step.
    pub v_mid: u8,

    /// Vhigh is used to specify the motor bemf drive levels at the maximum speed step.
    pub v_high: u8,
}

/// Builds a speed table from the specified configuration.
///
/// The speed table contains back-emf levels for a given throttle level (the index).
pub fn build(config: Config) -> SpeedTable {
    // This method uses floating point operations. The intention is it isn't
    // called often. If this changes, we should implement integer approximations.

    // Segment 1 (indices 1..=63): v_min -> v_mid
    // Segment 2 (indices 64..=126): v_mid -> v_max

    // don't multiply by 16 here as we want higher resolution at low speeds.
    let v_min = config.v_start as f32;

    // multiply by 16 to scale the values to the 12-bit adc range.
    let v_mid = config.v_mid as f32 * 16.0;
    let v_max = config.v_high as f32 * 16.0;

    let mut table = [0u16; SPEED_TABLE_LEN];

    // first entry is always 0, as it is no power
    table[0] = 0;

    let calc_step = |start: f32, end: f32, idx: usize| -> u16 {
        let m = (end - start) / 63.0;
        let val = m * idx as f32 + start;
        debug_assert!(val >= 0.0 && val <= u16::MAX as f32);
        // Round to nearest to match the C reference's lround() (core1.c). The value is
        // non-negative here, so adding 0.5 before the truncating cast is round-half-up.
        (val.clamp(0.0, u16::MAX as f32) + 0.5) as u16
    };

    // First segment (indices 1..=63): linear from v_min to v_mid
    const MIDPOINT: usize = SPEED_TABLE_LEN / 2;
    table
        .iter_mut()
        .enumerate()
        .skip(1)
        .take(MIDPOINT)
        .for_each(
            |(i, v)| *v = calc_step(v_min, v_mid, i - 1), // -1 because we skip the first entry
        );

    // Second segment (indices 64..=126): linear from v_mid to v_max
    table.iter_mut().enumerate().skip(MIDPOINT + 1).for_each(
        |(i, v)| *v = calc_step(v_mid, v_max, i - MIDPOINT), // skip the first half
    );

    table
}

#[cfg(test)]
mod tests {
    use super::*;

    // Helper function that mirrors the rounding used in build() for expectations
    fn interp(start: i32, end: i32, step: i32) -> u8 {
        let num = (end - start) * step + start * 63;
        let val = (num + 31) / 63;
        val.clamp(0, 255) as u8
    }

    #[test]
    fn table_matches_index_in_simple_first_segment_case() {
        // Choose v_start=1 and v_mid=64 so the first segment (indices 1..=63)
        // evaluates exactly to the index value due to the rounding scheme.
        let cfg = Config {
            v_start: 1,
            v_mid: 64,
            v_high: 126,
        };
        let table = build(cfg);

        // index 0 is always 0
        assert_eq!(
            table.as_slice(),
            &[
                0, 1, 17, 33, 50, 66, 82, 98, 115, 131, 147, 163, 180, 196, 212, 228, 245, 261,
                277, 293, 310, 326, 342, 358, 374, 391, 407, 423, 439, 456, 472, 488, 504, 521,
                537, 553, 569, 586, 602, 618, 634, 651, 667, 683, 699, 715, 732, 748, 764, 780,
                797, 813, 829, 845, 862, 878, 894, 910, 927, 943, 959, 975, 992, 1008, 1040, 1055,
                1071, 1087, 1103, 1118, 1134, 1150, 1166, 1181, 1197, 1213, 1229, 1244, 1260, 1276,
                1292, 1307, 1323, 1339, 1355, 1370, 1386, 1402, 1418, 1433, 1449, 1465, 1481, 1496,
                1512, 1528, 1544, 1559, 1575, 1591, 1607, 1622, 1638, 1654, 1670, 1685, 1701, 1717,
                1733, 1748, 1764, 1780, 1796, 1811, 1827, 1843, 1859, 1874, 1890, 1906, 1922, 1937,
                1953, 1969, 1985, 2000, 2016
            ]
        );

        assert_eq!(table[0], 0);
        assert_eq!(table[126], 126 * 16);
    }

    #[test]
    fn two_segment_table_with_different_slopes() {
        // Slopes: (50-10) != (200-50)
        let cfg = Config {
            v_start: 10,
            v_mid: 50,
            v_high: 200,
        };

        // Capture values before moving cfg into build()
        let v_start = cfg.v_start as i32;
        let v_mid = cfg.v_mid as i32;
        let v_high = cfg.v_high as i32;
        let table = build(cfg);

        // First segment endpoints and some midpoints
        assert_eq!(table[1], 10);
        assert_eq!(table[63], 787);

        // Second segment start, midpoint step, and end
        assert_eq!(table[64], 838);
        assert_eq!(table[126], 200 * 16); // should be exactly v_high
    }
}
