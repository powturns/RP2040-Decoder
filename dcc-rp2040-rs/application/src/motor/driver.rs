use crate::MOTOR_CHANNEL;
use crate::motor::{Command, MotorDriver, RpMotorDriver};
use ::motor::speed::Controller as SpeedController;
use ::motor::speed::accel;
use core::cmp::min;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use motor::speed::table::SpeedTable;
use motor::{Direction, VelocitySetpoint};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Error {
    Motor(crate::motor::Error),
    Speed(::motor::speed::Error),
}

impl From<crate::motor::Error> for Error {
    fn from(value: crate::motor::Error) -> Self {
        Self::Motor(value)
    }
}

impl From<::motor::speed::Error> for Error {
    fn from(value: ::motor::speed::Error) -> Self {
        Self::Speed(value)
    }
}

pub(crate) fn spawn(
    spawner: Spawner,
    pid_sample_time: Duration,
    motor_driver: RpMotorDriver,
    speed_control: SpeedController,
    accel_helper: accel::Helper,
    speed_table: SpeedTable,
) {
    spawner.spawn(unwrap!(accel_helper_task(accel_helper, speed_table)));
    spawner.spawn(unwrap!(motor_driver_task(
        pid_sample_time,
        motor_driver,
        speed_control,
    )));
    spawner.spawn(unwrap!(dispatcher()));
}

#[embassy_executor::task]
async fn dispatcher() {
    trace!("starting motor dispatcher task");
    loop {
        let command = MOTOR_CHANNEL.receive().await;
        match command {
            Command::AcknowledgeCv => {
                MOTOR_DRIVER_CHANNEL.send(command).await;
            }
            Command::Reset => {
                ACCEL_HELPER_CHANNEL.send(command).await;
                MOTOR_DRIVER_CHANNEL.send(command).await;
            }
            Command::SetVelocity128(_) => {
                ACCEL_HELPER_CHANNEL.send(command).await;
            }
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct OutputLevel {
    dir: Direction,
    setpoint: u16,
}

static MOTOR_DRIVER_CHANNEL: Channel<ThreadModeRawMutex, Command, 5> = Channel::new();
static MOTOR_SPEED_CHANNEL: Signal<ThreadModeRawMutex, OutputLevel> = Signal::new();
#[embassy_executor::task]
async fn motor_driver_task(
    pid_sample_time: Duration,
    mut motor_driver: RpMotorDriver,
    mut speed_control: SpeedController,
) {
    trace!("starting motor driver task");

    /// Measures the back emf, computes the new output and applies it.
    ///
    /// `tick` is the scheduled time of this control step, not the current time. The speed
    /// controller is a fixed-rate discrete controller: it always assumes one sample period
    /// between steps, and skips a step if its timestamp is less than one sample period after the
    /// previous one. Passing the scheduled time means jitter in when a step actually runs can
    /// never cause a step to be skipped.
    async fn compute_and_set_output(
        motor_driver: &mut RpMotorDriver,
        speed_control: &mut SpeedController,
        level: OutputLevel,
        tick: Instant,
    ) -> Result<(), Error> {
        let bemf = motor_driver.measure().await?;

        let output = speed_control.compute(bemf, level.setpoint as u32, tick.as_millis())?;

        motor_driver.set_output_level(output, level.dir)?;

        Ok(())
    }

    let mut output_level = OutputLevel {
        dir: Direction::Forward,
        setpoint: 0,
    };

    // Scheduled time of the next control step, or None when the motor is stopped. Steps are
    // scheduled at a fixed rate (`next_tick += pid_sample_time`), so the time spent measuring and
    // computing doesn't stretch the sample period. The scheduled times are also the timestamps
    // passed to the speed controller, so they must only ever increase by at least one period.
    let mut next_tick: Option<Instant> = None;

    loop {
        let tick_timer = Timer::at(next_tick.unwrap_or(Instant::MAX));

        match select3(
            MOTOR_SPEED_CHANNEL.wait(),
            MOTOR_DRIVER_CHANNEL.receive(),
            tick_timer,
        )
        .await
        {
            Either3::First(level) => {
                trace!("MOTOR_SPEED_CHANNEL: level={:?}", level);

                let dir_changed = motor_driver.dir != level.dir;
                output_level = level;

                if output_level.setpoint == 0 || dir_changed {
                    // The setpoint is zero, or we just changed direction. Reset the speed
                    // controller and stop the motor immediately. Use set_output_level so the new
                    // direction is set.
                    speed_control.reset();
                    let _ = motor_driver
                        .set_output_level(0, output_level.dir)
                        .inspect_err(|e| error!("error stopping motor: {:?}", e));
                }

                if output_level.setpoint == 0 {
                    // Don't run the tick timer until the setpoint changes off of zero.
                    next_tick = None;
                } else if next_tick.is_none() {
                    // The setpoint is non-zero, but the tick timer is not running. The motor
                    // was previously stopped. Now that we're running, we need to ensure we're
                    // sampling the BEMF and adjusting accordingly.
                    next_tick = Some(Instant::now() + pid_sample_time);
                }
            }
            Either3::Second(c) => match c {
                // command received via a packet
                Command::AcknowledgeCv => {
                    let _ = motor_driver
                        .acknowledge_cv()
                        .await
                        .inspect_err(|e| error!("error acknowledging cv: {:?}", e));

                    next_tick = None;
                }
                Command::Reset => {
                    let _ = motor_driver
                        .stop()
                        .inspect_err(|e| error!("error stopping motor: {:?}", e));

                    speed_control.reset();

                    next_tick = None;
                }
                Command::SetVelocity128(_) => {
                    // noop - handled by the speed channel
                }
            },
            Either3::Third(_) => {
                // Control step. The timer only runs while the setpoint is non-zero.
                let Some(tick) = next_tick else { continue };

                let _ = compute_and_set_output(
                    &mut motor_driver,
                    &mut speed_control,
                    output_level,
                    tick,
                )
                .await
                .inspect_err(|e| error!("error setting output: {:?}", e));

                let mut tick_candidate = tick + pid_sample_time;
                let now = Instant::now();
                if tick_candidate <= now {
                    // the tick candidate is in the past! This means the executor was doing too
                    // much work and couldn't execute the tick within the sample period.
                    // Instead of executing the (late) tick immediately, skip it and schedule
                    // a new one after a full period.
                    warn!(
                        "motor control step late by {}us",
                        (now - tick_candidate).as_micros()
                    );
                    tick_candidate = now + pid_sample_time;
                }
                next_tick = Some(tick_candidate);
            }
        }
    }
}

static ACCEL_HELPER_CHANNEL: Channel<ThreadModeRawMutex, Command, 5> = Channel::new();
#[embassy_executor::task]
async fn accel_helper_task(mut accel_helper: accel::Helper, speed_table: SpeedTable) {
    trace!("starting accel helper task");

    // convert the velocity to an output level by indexing into the speed table.
    let to_output_level = |v: VelocitySetpoint| OutputLevel {
        dir: v.direction,
        setpoint: {
            let idx = min(v.speed_step.idx() as usize, speed_table.len() - 1);
            speed_table[idx]
        },
    };

    let mut previous_setpoint: Option<VelocitySetpoint> = None;
    // Sends the output level to the speed channel if the setpoint has changed.
    let mut maybe_send_to_channel = |setpoint: VelocitySetpoint| {
        if previous_setpoint != Some(setpoint) {
            previous_setpoint = Some(setpoint);
            MOTOR_SPEED_CHANNEL.signal(to_output_level(setpoint));
        }
    };

    let mut accel_timer = Timer::at(Instant::MAX);

    loop {
        match select(ACCEL_HELPER_CHANNEL.receive(), &mut accel_timer).await {
            Either::First(c) => match c {
                Command::AcknowledgeCv => { /*noop*/ }
                Command::Reset => {
                    accel_helper.reset();

                    // reset the motor speed channel to stopped to ensure the previous_setpoint
                    // value is accurate
                    maybe_send_to_channel(VelocitySetpoint::default());
                    accel_timer = Timer::at(Instant::MAX);
                }
                Command::SetVelocity128(v) => {
                    accel_helper.set_target(v);
                    let (setpoint, delay) = accel_helper.step();
                    maybe_send_to_channel(setpoint);
                    accel_timer = duration_to_timer(delay.try_into().unwrap_or(Duration::MAX))
                }
            },
            Either::Second(_) => {
                // accel_timer expired
                let (setpoint, delay) = accel_helper.step();
                maybe_send_to_channel(setpoint);
                accel_timer = duration_to_timer(delay.try_into().unwrap_or(Duration::MAX))
            }
        }
    }
}

/// Safely converts the duration to the timer.
pub fn duration_to_timer(d: Duration) -> Timer {
    Timer::at(Instant::now().saturating_add(d))
}
