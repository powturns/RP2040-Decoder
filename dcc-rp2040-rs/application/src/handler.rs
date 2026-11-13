use crate::functions::handler::Handler as FunctionGroupHandler;
use crate::motor::Command;
use crate::timer::InstantTimer;
use crate::{MOTOR_CHANNEL, PACKET_CHANNEL, store};
use ::motor::speed::calibration;
use dcc::cv::store::StoreExt;
use dcc::handler::{Handler, Op};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

pub(crate) type Packethandler = Handler<InstantTimer, store::CvStore>;

/// Commands sent from other tasks back to the packet handler, which owns the CV store.
///
/// This is the place to add new variants whenever a task needs the control loop (the store
/// owner) to do something on its behalf, rather than adding a dedicated static per case.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum ControlCommand {
    /// The outcome of a motor calibration run. `Ok` carries the offset-corrected BEMF at
    /// full output.
    MotorCalibrationResult(Result<u16, calibration::Failure>),
}

pub(crate) static CONTROL_CHANNEL: Channel<ThreadModeRawMutex, ControlCommand, 4> = Channel::new();

/// Dispatches a command sent to the packet handler by another task.
async fn handle_control_command(handler: &mut Packethandler, command: ControlCommand) {
    match command {
        ControlCommand::MotorCalibrationResult(result) => {
            handle_calibration_result(handler, result).await
        }
    }
}

/// Handles the outcome of a motor calibration run. On success the new CV5 / CV6 values are
/// computed and stored, and the decoder reboots so the speed table is rebuilt. On failure only the
/// status is stored and the decoder keeps running.
async fn handle_calibration_result(
    handler: &mut Packethandler,
    result: Result<u16, calibration::Failure>,
) {
    let failure = match result {
        Ok(emf_max) => {
            let store = handler.store();
            let params = async {
                Ok::<_, dcc::cv::store::Error>((
                    store.motor_calibration_headroom().await?,
                    store.v_high().await?,
                    store.v_mid().await?,
                ))
            }
            .await;

            let (headroom, v_high_old, v_mid_old) = match params {
                Ok(p) => p,
                Err(e) => {
                    error!("error reading calibration parameters: {:?}", e);
                    return;
                }
            };

            match calibration::compute_cvs(emf_max, headroom, v_high_old, v_mid_old) {
                Ok(c) => {
                    info!(
                        "motor calibration: emf_max={} v_high={} v_mid={}",
                        emf_max, c.v_high, c.v_mid
                    );

                    if handler
                        .store_mut()
                        .write_motor_calibration_result(
                            emf_max,
                            c.v_high,
                            c.v_mid,
                            calibration::STATUS_OK,
                        )
                        .await
                        .inspect_err(|e| error!("error storing calibration result: {:?}", e))
                        .is_ok()
                    {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    return;
                }
                Err(f) => f,
            }
        }
        Err(f) => f,
    };

    warn!("motor calibration failed: {:?}", failure);
    let _ = handler
        .store_mut()
        .write_motor_calibration_status(failure.status())
        .await
        .inspect_err(|e| error!("error storing calibration status: {:?}", e));
}

#[embassy_executor::task]
pub(crate) async fn packet_handler(
    mut handler: Packethandler,
    mut fg_handler: FunctionGroupHandler,
) {
    trace!("starting packet handler loop");

    let mut receiver = unwrap!(PACKET_CHANNEL.subscriber());

    let mut last_command: Option<Op> = None;

    loop {
        // TODO: if a packet hasn't been received within a heartbeat timeout
        //       we should execute a reset operation.
        //       We should probably do the same if we haven received a packet addressed to us
        //       in a while
        let packet = match select(receiver.next_message_pure(), CONTROL_CHANNEL.receive()).await {
            Either::First(packet) => packet,
            Either::Second(command) => {
                handle_control_command(&mut handler, command).await;
                continue;
            }
        };

        if cfg!(feature = "verbose-transport") {
            trace!("handling packet: {:?}", packet);
        }

        let op = handler.handle(&packet).await;

        if cfg!(feature = "verbose-transport") {
            trace!("packet result: {:?}", op);
        }

        match op {
            Ok(Some(op)) => {
                match op {
                    Op::AcknowledgeCv => MOTOR_CHANNEL.send(Command::AcknowledgeCv).await,
                    Op::Reset => MOTOR_CHANNEL.send(Command::Reset).await,
                    Op::Velocity128(sp) => {
                        if last_command != Some(op) {
                            // only send the velocity command if something changed. This prevents
                            // unnecessary work from being performed downstream
                            MOTOR_CHANNEL.send(Command::SetVelocity128(sp)).await;

                            fg_handler.set_direction(sp.direction);
                        }
                    }
                    Op::SetFunctions(fg) => fg_handler.handle(fg),
                    Op::CalibrateMotor => MOTOR_CHANNEL.send(Command::Calibrate).await,
                    Op::Reboot => {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                }
                last_command = Some(op);
            }
            Ok(None) => { /* noop */ }
            Err(e) => {
                error!("error handling packet: {:?} {:?}", e, packet);
            }
        }
    }
}
