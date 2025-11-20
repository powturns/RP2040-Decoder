const SPEED_TABLE_LEN: usize = 127;

pub struct Config {
    /// Used to define the voltage drive level used as the start voltage on the motor.
    pub v_start: u8,

    /// Vmid specifies the voltage drive level at the middle speed step.
    pub v_mid: u8,

    /// Vhigh is used to specify the motor voltage drive levels at the maximum speed step.
    pub v_high: u8,
}

/// Builds a speed table from the specified configuration.
pub fn build(config: Config) -> [u16; SPEED_TABLE_LEN] {
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
        val.clamp(0.0, u16::MAX as f32) as u16
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
                0, 1, 17, 33, 49, 65, 82, 98, 114, 130, 147, 163, 179, 195, 212, 228, 244, 260,
                277, 293, 309, 325, 341, 358, 374, 390, 406, 423, 439, 455, 471, 488, 504, 520,
                536, 553, 569, 585, 601, 618, 634, 650, 666, 682, 699, 715, 731, 747, 764, 780,
                796, 812, 829, 845, 861, 877, 894, 910, 926, 942, 959, 975, 991, 1007, 1039, 1055,
                1071, 1086, 1102, 1118, 1134, 1149, 1165, 1181, 1197, 1212, 1228, 1244, 1260, 1275,
                1291, 1307, 1323, 1338, 1354, 1370, 1386, 1401, 1417, 1433, 1449, 1464, 1480, 1496,
                1512, 1527, 1543, 1559, 1575, 1590, 1606, 1622, 1638, 1653, 1669, 1685, 1701, 1716,
                1732, 1748, 1764, 1779, 1795, 1811, 1827, 1842, 1858, 1874, 1890, 1905, 1921, 1937,
                1953, 1968, 1984, 2000, 2016
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
