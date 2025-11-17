
/// Calculates the mean of the [data] filtering out any elements that are
/// more than [std_deviation] away from the mean.
fn filtered_mean(data: &[u16], std_deviation: u16) {
    let mean: u64 = data.iter().map(|&e| e as u64).sum() / data.len();

    // calculate standard deviation
    let variance : u16 = data.iter().map(|&e| (e as u64 - mean).pow(2)).sum() / (data.len() - 1);
    let std_dev = (variance as f32).sqrt() as u16;
}