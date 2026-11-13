use crate::MOTOR_CHANNEL;
use crate::handler::{CONTROL_CHANNEL, ControlCommand};
use crate::motor::{Command, MotorDriver, RpMotorDriver};
use ::motor::speed::Controller as SpeedController;
use ::motor::speed::accel;
use ::motor::speed::calibration;
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
    calibration: calibration::Config,
) {
    spawner.spawn(unwrap!(accel_helper_task(accel_helper, speed_table)));
    spawner.spawn(unwrap!(motor_driver_task(
        pid_sample_time,
        motor_driver,
        speed_control,
        calibration,
    )));
    spawner.spawn(unwrap!(dispatcher()));
}

/// Dispatches motor commands from outside the module to the appropriate
/// in-module channels.
#[embassy_executor::task]
async fn dispatcher() {
    trace!("starting motor dispatcher task");
    loop {
        let command = MOTOR_CHANNEL.receive().await;
        match command {
            Command::AcknowledgeCv => {
                MOTOR_COMMAND_CHANNEL.send(command).await;
            }
            Command::Reset => {
                ACCEL_HELPER_CHANNEL.send(command).await;
                MOTOR_COMMAND_CHANNEL.send(command).await;
            }
            Command::Calibrate => {
                MOTOR_COMMAND_CHANNEL.send(command).await;
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

/// Channel for non-speed control commands.
static MOTOR_COMMAND_CHANNEL: Channel<ThreadModeRawMutex, Command, 5> = Channel::new();

/// Channel for motor speed control.
static MOTOR_SPEED_CHANNEL: Signal<ThreadModeRawMutex, OutputLevel> = Signal::new();

/// Reports the outcome of a calibration run to the packet handler.
///
/// This is a non-blocking send: the motor task must not stall its control loop, and a blocking
/// send could deadlock in a cycle with the packet handler, which itself awaits
/// `MOTOR_CHANNEL.send` towards this task. Unlike a `Signal`, the channel queues results in
/// order instead of overwriting.
fn report_calibration(result: Result<u16, calibration::Failure>) {
    if CONTROL_CHANNEL
        .try_send(ControlCommand::MotorCalibrationResult(result))
        .is_err()
    {
        error!("control channel full, dropping calibration result");
    }
}

/// State owned by the motor driver task.
struct DriverState {
    pid_sample_time: Duration,
    motor_driver: RpMotorDriver,
    speed_control: SpeedController,
    calibration: calibration::Config,

    output_level: OutputLevel,

    // Scheduled time of the next control step, or None when the motor is stopped. Steps are
    // scheduled at a fixed rate (`next_tick += pid_sample_time`), so the time spent measuring and
    // computing doesn't stretch the sample period. The scheduled times are also the timestamps
    // passed to the speed controller, so they must only ever increase by at least one period.
    next_tick: Option<Instant>,

    // Some while a BEMF calibration run is in progress. The run is open-loop and uses the same
    // tick as the speed controller.
    calibrating: Option<calibration::Calibrator>,
    // Control steps since the calibration run started. Only used to thin out the log.
    calibration_ticks: u32,
}

impl DriverState {
    fn new(
        pid_sample_time: Duration,
        motor_driver: RpMotorDriver,
        speed_control: SpeedController,
        calibration: calibration::Config,
    ) -> Self {
        Self {
            pid_sample_time,
            motor_driver,
            speed_control,
            calibration,
            output_level: OutputLevel {
                dir: Direction::Forward,
                setpoint: 0,
            },
            next_tick: None,
            calibrating: None,
            calibration_ticks: 0,
        }
    }

    /// Stops the motor, resets the speed controller and stops the control steps.
    fn stop(&mut self) {
        let _ = self
            .motor_driver
            .stop()
            .inspect_err(|e| error!("error stopping motor: {:?}", e));
        self.speed_control.reset();
        self.next_tick = None;
    }

    /// Aborts a running calibration, if there is one: stops the motor, clears the calibration
    /// state and reports the abort. The caller continues with whatever event caused the abort.
    fn abort_calibration(&mut self) {
        if self.calibrating.take().is_none() {
            return;
        }
        self.stop();
        warn!("motor calibration aborted");
        report_calibration(Err(calibration::Failure::Aborted));
    }

    fn set_output_level(&mut self, level: OutputLevel) {
        // a throttle change aborts the calibration, then we follow the throttle.
        self.abort_calibration();

        let dir_changed = self.motor_driver.dir != level.dir;
        self.output_level = level;

        if self.output_level.setpoint == 0 || dir_changed {
            // The setpoint is zero, or we just changed direction. Reset the speed
            // controller and stop the motor immediately. Use set_output_level so the new
            // direction is set.
            self.speed_control.reset();
            let _ = self
                .motor_driver
                .set_output_level(0, self.output_level.dir)
                .inspect_err(|e| error!("error stopping motor: {:?}", e));
        }

        if self.output_level.setpoint == 0 {
            // Don't run the tick timer until the setpoint changes off of zero.
            self.next_tick = None;
        } else if self.next_tick.is_none() {
            // The setpoint is non-zero, but the tick timer is not running. The motor
            // was previously stopped. Now that we're running, we need to ensure we're
            // sampling the BEMF and adjusting accordingly.
            self.next_tick = Some(Instant::now() + self.pid_sample_time);
        }
    }

    async fn handle_command(&mut self, command: Command) {
        match command {
            Command::AcknowledgeCv => {
                self.abort_calibration();

                let _ = self
                    .motor_driver
                    .acknowledge_cv()
                    .await
                    .inspect_err(|e| error!("error acknowledging cv: {:?}", e));

                self.next_tick = None;
            }
            Command::Reset => {
                self.abort_calibration();
                self.stop();
            }
            Command::Calibrate => self.start_calibration(),
            Command::SetVelocity128(_) => {
                // noop - handled by the speed channel
            }
        }
    }

    fn start_calibration(&mut self) {
        if self.calibrating.is_some() {
            // already running
        } else if self.output_level.setpoint != 0 || self.next_tick.is_some() {
            warn!("motor calibration refused: the loco is not stopped");
            report_calibration(Err(calibration::Failure::Aborted));
        } else {
            info!("starting motor calibration");
            self.speed_control.reset();
            // latch the forward direction. measure() uses the last set direction.
            let _ = self
                .motor_driver
                .set_output_level(0, Direction::Forward)
                .inspect_err(|e| error!("error stopping motor: {:?}", e));
            self.calibrating = Some(calibration::Calibrator::new(self.calibration));
            self.calibration_ticks = 0;
            self.next_tick = Some(Instant::now() + self.pid_sample_time);
        }
    }

    /// Runs the control step scheduled for `tick`, then schedules the next one.
    async fn control_step(&mut self, tick: Instant) {
        if self.calibrating.is_some() {
            if !self.calibration_step().await {
                // the run is over and the motor is stopped.
                // No need for more ticks until there's new input.
                self.next_tick = None;
                return;
            }
        } else {
            let _ = self
                .compute_and_set_output(tick)
                .await
                .inspect_err(|e| error!("error setting output: {:?}", e));
        }

        // schedule the next control step
        let mut tick_candidate = tick + self.pid_sample_time;
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
            tick_candidate = now + self.pid_sample_time;
        }
        self.next_tick = Some(tick_candidate);
    }

    /// Runs one step of the calibration. Returns false once the run is done.
    async fn calibration_step(&mut self) -> bool {
        let Some(cal) = self.calibrating.as_mut() else {
            return false;
        };

        let step = match self.motor_driver.measure().await {
            Ok(bemf) => {
                let step = cal.step(bemf);
                // SWD usually drops out during a run, so keep this sparse enough
                // to still fit in the RTT buffer when read back afterwards.
                if self.calibration_ticks % 20 == 0 {
                    debug!(
                        "calibration(tick={}, measurement={}, step={:?})",
                        self.calibration_ticks, bemf, step
                    );
                }
                self.calibration_ticks += 1;
                step
            }
            Err(e) => {
                error!("error measuring bemf during calibration: {:?}", e);
                calibration::Step::Done(Err(calibration::Failure::Aborted))
            }
        };

        match step {
            calibration::Step::Output(out) => {
                let _ = self
                    .motor_driver
                    .set_output_level(out, Direction::Forward)
                    .inspect_err(|e| error!("error setting output: {:?}", e));
                true
            }
            calibration::Step::Done(result) => {
                self.calibrating = None;
                self.stop();
                report_calibration(result);
                false
            }
        }
    }

    /// Measures the back emf, computes the new output and applies it.
    ///
    /// `tick` is the scheduled time of this control step, not the current time. The speed
    /// controller is a fixed-rate discrete controller: it always assumes one sample period
    /// between steps, and skips a step if its timestamp is less than one sample period after the
    /// previous one. Passing the scheduled time means jitter in when a step actually runs can
    /// never cause a step to be skipped.
    async fn compute_and_set_output(&mut self, tick: Instant) -> Result<(), Error> {
        let bemf = self.motor_driver.measure().await?;

        let output = self.speed_control.compute(
            bemf,
            self.output_level.setpoint as u32,
            tick.as_millis(),
        )?;

        self.motor_driver
            .set_output_level(output, self.output_level.dir)?;

        Ok(())
    }
}

#[embassy_executor::task]
async fn motor_driver_task(
    pid_sample_time: Duration,
    motor_driver: RpMotorDriver,
    speed_control: SpeedController,
    calibration: calibration::Config,
) {
    trace!("starting motor driver task");

    let mut state = DriverState::new(pid_sample_time, motor_driver, speed_control, calibration);

    loop {
        // Resolves with the scheduled tick when it arrives. Never resolves while stopped.
        let next_tick = state.next_tick;
        let tick_timer = async move {
            match next_tick {
                Some(tick) => {
                    Timer::at(tick).await;
                    tick
                }
                None => core::future::pending().await,
            }
        };

        match select3(
            MOTOR_SPEED_CHANNEL.wait(),
            MOTOR_COMMAND_CHANNEL.receive(),
            tick_timer,
        )
        .await
        {
            Either3::First(level) => {
                trace!("MOTOR_SPEED_CHANNEL: level={:?}", level);
                state.set_output_level(level);
            }
            Either3::Second(command) => state.handle_command(command).await,
            // Control step. Runs while the motor is driven or a calibration is in progress.
            Either3::Third(tick) => state.control_step(tick).await,
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
                Command::Calibrate => { /*noop*/ }
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
                    accel_timer = duration_to_timer(delay)
                }
            },
            Either::Second(_) => {
                // accel_timer expired
                let (setpoint, delay) = accel_helper.step();
                maybe_send_to_channel(setpoint);
                accel_timer = duration_to_timer(delay)
            }
        }
    }
}

/// Safely converts the duration to the timer.
pub fn duration_to_timer(d: Duration) -> Timer {
    Timer::at(Instant::now().saturating_add(d))
}
