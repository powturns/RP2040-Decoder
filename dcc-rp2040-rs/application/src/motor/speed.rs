// use embassy_futures::select::{select, Either};
// use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
// use embassy_sync::channel::Channel;
// use embassy_sync::watch::{Receiver, Sender, Watch};
// use embassy_time::{Delay, Duration, Timer};
// use crate::motor::Direction;
//
// #[derive(Eq, PartialEq, Copy, Clone)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// enum SpeedStep {
//     Stop,
//     EmergencyStop,
//     Num(u8),
// }
//
// impl SpeedStep {
//     fn idx(&self) -> u8 {
//         match self {
//             SpeedStep::Stop |
//             SpeedStep::EmergencyStop => 0,
//             SpeedStep::Num(n) => *n
//         }
//     }
// }
//
// struct PackedSpeedStep {
//     step: u8,
// }
//
// impl PackedSpeedStep {
//     /// Returns the direction encoded in this control step.
//     /// According to the C implementation (see `shared.h`), the MSB encodes direction:
//     /// - 0 => Reverse (0x00..0x7F)
//     /// - 1 => Forward (0x80..0xFF)
//     pub fn direction(&self) -> Direction {
//         if (self.step & 0x80) != 0 { Direction::Forward } else { Direction::Reverse }
//     }
//
//     /// Returns true if this control step encodes a STOP command.
//     /// STOP is defined as value where the lower 7 bits are 0 (0x00 or 0x80).
//     pub fn is_stop(&self) -> bool {
//         (self.step & 0x7F) == 0
//     }
//
//     /// Returns true if this control step encodes an EMERGENCY STOP command.
//     /// EMERGENCY STOP is defined as value where the lower 7 bits are 1 (0x01 or 0x81).
//     pub fn is_emergency_stop(&self) -> bool {
//         (self.step & 0x7F) == 1
//     }
//
//     /// Returns the numeric control step (1..=126) for regular control values.
//     /// Returns None for STOP and EMERGENCY STOP.
//     pub fn num(&self) -> Option<u8> {
//         let raw = self.step & 0x7F; // strip direction bit
//         if raw >= 2 { Some(raw - 1) } else { None }
//
//         // TODO: consider returning an enum
//     }
// }
//
// #[derive(Eq, PartialEq, Copy, Clone)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// struct SpeedTarget {
//     speed_step: SpeedStep,
//     direction: Direction,
// }
//
// struct Config {
//     // CV3
//     accel_rate: u8,
//     // CV4
//     decel_rate: u8,
//
//     // CV175
//     loop_delay: Duration, // FIXME: default to 7ms
// }
//
// struct SpeedControl {
//     config: Config,
// }
//
// impl SpeedControl {
//     fn new(config: Config) -> Self {
//         Self {
//             config
//         }
//     }
//
//     async fn run<'a, M: RawMutex, const RXN: usize, const TXN: usize,>(
//         &mut self,
//         mut rx: Receiver<'a, M, SpeedTarget, RXN>,
//         tx: Sender<'a, M, SpeedTarget, TXN>,
//     )
//     -> Result<(), ()> {
//         let mut target = SpeedTarget {
//             speed_step: SpeedStep::Stop,
//             direction: Direction::Forward,
//         };
//         let mut current = target.clone();
//
//         loop {
//
//             let accelerating = current.speed_step.idx() < target.speed_step.idx();
//             let decelerating = current.speed_step.idx() > target.speed_step.idx();
//
//             // calculate the new control idx
//             if let SpeedStep::EmergencyStop = target.speed_step {
//                 // upon emergency stop, immediately reset the control
//                 current.speed_step = SpeedStep::EmergencyStop;
//             } else if current.direction != target.direction {
//                 // jump immediately to the new direction
//                 // FIXME: this should probably be gradual?
//                 current.speed_step = target.speed_step;
//                 current.direction = target.direction;
//             } else {
//                 // ramp to the new control if there is an accel/decel rate,
//                 // otherwise jump directly to new control.
//                 if accelerating {
//                     current.speed_step = SpeedStep::Num(
//                         if self.config.accel_rate > 0 {current.speed_step.idx() + 1 } else {target.speed_step.idx()}
//                     );
//                 } else if decelerating {
//                     current.speed_step = SpeedStep::Num(
//                         if self.config.decel_rate > 0 {current.speed_step.idx() - 1} else {target.speed_step.idx()}
//                     )
//                 }
//             }
//
//             // publish control step if it's changed
//             tx.send_if_modified(|watch_val| {
//                 if let Some(v) = watch_val && v != &current {
//                     *watch_val = Some(current);
//                     true
//                 } else {
//                     false
//                 }
//             });
//
//             // calculate the new delay
//             // Standard formula is CV#3*.896)/(number of control steps in use).
//             // Time for 1 Speed Step := (speed_helper timer delay)*(accel_rate) or CV_175*CV_3 or CV_175*CV_4
//             let step_delay = if accelerating {
//                 // accelerating
//                 self.config.accel_rate as u32 * self.config.loop_delay
//             } else if decelerating {
//                 // decelerating
//                 self.config.decel_rate as u32 * self.config.loop_delay
//             } else {
//                 // At target - wait indefinitely for new target
//                 Duration::MAX
//             };
//
//             match select(rx.changed(), Timer::after(step_delay)).await {
//                 Either::First(t) => {
//                     target = t
//                 },
//                 Either::Second(_) => {
//                     // timer expired
//                 }
//             }
//         }
//     }
// }