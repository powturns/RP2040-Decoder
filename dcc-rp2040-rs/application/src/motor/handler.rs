use crate::MOTOR_CHANNEL;
use crate::motor::{Command, Controller as MotorController, RpMotorController};
use crate::timers::{INSTANT_MAX_FIXED, duration_to_timer_saturating};
use ::motor::speed::Controller as SpeedController;
use ::motor::speed::accel;
use core::cmp::min;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_rp::peripherals::DMA_CH2;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
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
    motor_controller: RpMotorController<DMA_CH2>,
    speed_control: SpeedController,
    accel_helper: accel::Helper,
    speed_table: SpeedTable,
) {
    spawner.must_spawn(accel_helper_task(accel_helper, speed_table));
    spawner.must_spawn(motor_controller_task(
        pid_sample_time,
        motor_controller,
        speed_control,
    ));
    spawner.must_spawn(dispatcher());
}

#[embassy_executor::task]
async fn dispatcher() {
    trace!("starting motor dispatcher task");
    loop {
        let command = MOTOR_CHANNEL.receive().await;
        match command {
            Command::AcknowledgeCv => {
                MOTOR_CONTROLLER_CHANNEL.send(command).await;
            }
            Command::Reset => {
                ACCEL_HELPER_CHANNEL.send(command).await;
                MOTOR_CONTROLLER_CHANNEL.send(command).await;
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

static MOTOR_CONTROLLER_CHANNEL: Channel<ThreadModeRawMutex, Command, 5> = Channel::new();
static MOTOR_SPEED_CHANNEL: Signal<ThreadModeRawMutex, OutputLevel> = Signal::new();
#[embassy_executor::task]
async fn motor_controller_task(
    pid_sample_time: Duration,
    mut motor_controller: RpMotorController<DMA_CH2>,
    mut speed_control: SpeedController,
) {
    trace!("starting motor controller task");
    async fn compute_and_set_output(
        motor_controller: &mut RpMotorController<DMA_CH2>,
        speed_control: &mut SpeedController,
        level: OutputLevel,
    ) -> Result<(), Error> {
        let bemf = motor_controller.measure().await?;

        let output =
            speed_control.compute(bemf, level.setpoint as u32, Instant::now().as_millis())?;

        // trace!("bemf={}, output={}", bemf, output);

        // fixme: consider calling this only if the level changes
        if let Some(output) = output {
            motor_controller.set_output_level(output, level.dir)?
        }

        Ok(())
    }

    let mut pid_sample_timer: Timer = Timer::at(INSTANT_MAX_FIXED);
    let mut last_output_level = OutputLevel {
        dir: Direction::Forward,
        setpoint: 0,
    };

    let mut set_output = async |new_output_level: Option<OutputLevel>,
                                motor_controller: &mut RpMotorController<DMA_CH2>,
                                speed_control: &mut SpeedController,
                                pid_sample_timer: &mut Timer| {
        if let Some(new_output_level) = new_output_level {
            last_output_level = new_output_level;
        }

        // only calculate the new output if the setpoint is non-zero,
        // otherwise immediately stop the motor. This saves cycles if the
        // engine is stopped.
        if last_output_level.setpoint != 0 {
            let _ = compute_and_set_output(motor_controller, speed_control, last_output_level)
                .await
                .inspect_err(|e| error!("error setting output: {:?}", e));

            *pid_sample_timer = Timer::after(pid_sample_time);
        } else {
            speed_control.reset();
            let _ = motor_controller
                .stop()
                .inspect_err(|e| error!("error stopping motor: {:?}", e));
            *pid_sample_timer = Timer::at(INSTANT_MAX_FIXED);
        }
    };

    loop {
        match select3(
            MOTOR_SPEED_CHANNEL.wait(),
            MOTOR_CONTROLLER_CHANNEL.receive(),
            &mut pid_sample_timer,
        )
        .await
        {
            Either3::First(level) => {
                trace!("MOTOR_SPEED_CHANNEL: level={:?}", level);

                Timer::after(Duration::from_millis(1)).await;

                // output level changed
                set_output(
                    Some(level),
                    &mut motor_controller,
                    &mut speed_control,
                    &mut pid_sample_timer,
                )
                .await;
            }
            Either3::Second(c) => match c {
                // command received via a packet
                Command::AcknowledgeCv => {
                    let _ = motor_controller
                        .acknowledge_cv()
                        .await
                        .inspect_err(|e| error!("error acknowledging cv: {:?}", e));

                    pid_sample_timer = Timer::at(INSTANT_MAX_FIXED);
                }
                Command::Reset => {
                    let _ = motor_controller
                        .stop()
                        .inspect_err(|e| error!("error stopping motor: {:?}", e));

                    speed_control.reset();

                    pid_sample_timer = Timer::at(INSTANT_MAX_FIXED);
                }
                Command::SetVelocity128(_) => {
                    // noop - handled by the speed channel
                }
            },
            Either3::Third(_) => {
                // pid timer timeout
                set_output(
                    None,
                    &mut motor_controller,
                    &mut speed_control,
                    &mut pid_sample_timer,
                )
                .await;
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

    let mut accel_timer = Timer::at(INSTANT_MAX_FIXED);

    loop {
        match select(ACCEL_HELPER_CHANNEL.receive(), &mut accel_timer).await {
            Either::First(c) => match c {
                Command::AcknowledgeCv => { /*noop*/ }
                Command::Reset => {
                    accel_helper.reset();
                    accel_timer = Timer::at(INSTANT_MAX_FIXED);
                }
                Command::SetVelocity128(v) => {
                    accel_helper.set_target(v);
                    let (setpoint, delay) = accel_helper.step();
                    maybe_send_to_channel(setpoint);
                    accel_timer =
                        duration_to_timer_saturating(delay.try_into().unwrap_or(Duration::MAX))
                }
            },
            Either::Second(_) => {
                // accel_timer expired
                let (setpoint, delay) = accel_helper.step();
                maybe_send_to_channel(setpoint);
                accel_timer =
                    duration_to_timer_saturating(delay.try_into().unwrap_or(Duration::MAX))
            }
        }
    }
}
