use embassy_time::{Duration, Instant, Timer};

// Can't use Instant::MAX due to https://github.com/embassy-rs/embassy/issues/5017
pub const INSTANT_MAX_FIXED: Instant = Instant::from_ticks(u64::MAX - 1);

/// Safely converts the duration to the timer, returning [INSTANT_MAX_FIXED] if the duration
/// overflows.
pub fn duration_to_timer_saturating(d: Duration) -> Timer {
    let expires_at = Instant::now().saturating_add(d).min(INSTANT_MAX_FIXED);

    Timer::at(expires_at)
}
