use crate::{SpeedStep, VelocitySetpoint};
use core::time::Duration;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    // CV3
    pub accel_rate: u8,
    // CV4
    pub decel_rate: u8,

    // CV175
    pub loop_delay: Duration, // FIXME: default to 7ms
}

pub struct Helper {
    config: Config,
    target: VelocitySetpoint,
    current: VelocitySetpoint,
}

impl Helper {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            target: VelocitySetpoint::default(),
            current: VelocitySetpoint::default(),
        }
    }

    /// sets the new control target. Call [step] after this calling this as the previously
    /// returned delay may no longer be valid.
    pub fn set_target(&mut self, target: VelocitySetpoint) {
        self.target = target;
    }

    pub fn reset(&mut self) {
        self.current = VelocitySetpoint::default();
        self.target = VelocitySetpoint::default();
    }

    /// Calculates the new [VelocitySetpoint] after a single step.
    ///
    /// Returns the new target, and the delay before calling step again.
    pub fn step(&mut self) -> (VelocitySetpoint, Duration) {
        if self.current.equivalent(&self.target) {
            return (self.target, Duration::MAX);
        }

        if self.current.direction != self.target.direction {
            // we are switching direction. We decelerate to zero, then switch
            // direction, and start accelerating normally.
            if self.config.decel_rate == 0 || self.current.speed_step.idx() == 0 {
                // there is either no deceleration rate, or we are already stopped - time to
                // switch direction
                self.current.speed_step = SpeedStep::Stop;
                self.current.direction = self.target.direction;
            }
        }

        let accelerating = (self.current.speed_step.idx() < self.target.speed_step.idx())
            && self.current.direction == self.target.direction;

        self.current.speed_step = if let SpeedStep::EmergencyStop = self.target.speed_step {
            SpeedStep::EmergencyStop
        } else {
            let current_idx = self.current.speed_step.idx();

            if accelerating {
                SpeedStep::Num(if self.config.accel_rate > 0 {
                    current_idx + 1
                } else {
                    // there is no acceleration rate, so jump directly
                    // to the target
                    self.target.speed_step.idx()
                })
            } else {
                // we must be decelerating
                SpeedStep::Num(if self.config.decel_rate > 0 {
                    current_idx - 1
                } else {
                    // There is no deceleration rate, jump directly
                    // to the target
                    self.target.speed_step.idx()
                })
            }
        };

        // if, after updating, we've reached the target, we're done!
        if self.current.equivalent(&self.target) {
            return (self.target, Duration::MAX);
        }

        let step_delay = if accelerating {
            // accelerating
            self.config.accel_rate as u32 * self.config.loop_delay
        } else {
            // decelerating
            self.config.decel_rate as u32 * self.config.loop_delay
        };

        (self.current, step_delay)
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    #[cfg(test)]
    extern crate std;

    use super::*;
    use crate::Direction::{self, Forward};
    use crate::SpeedStep;
    use alloc::vec;
    use alloc::vec::Vec;

    fn default_config() -> Config {
        Config {
            accel_rate: 10,
            decel_rate: 10,
            loop_delay: Duration::from_millis(7),
        }
    }

    fn config_no_ramp() -> Config {
        Config {
            accel_rate: 0,
            decel_rate: 0,
            loop_delay: Duration::from_millis(7),
        }
    }

    #[test]
    fn test_speed_step_idx() {
        assert_eq!(SpeedStep::Stop.idx(), 0);
        assert_eq!(SpeedStep::EmergencyStop.idx(), 0);
        assert_eq!(SpeedStep::Num(5).idx(), 5);
        assert_eq!(SpeedStep::Num(126).idx(), 126);
    }

    #[test]
    fn test_speed_target_default() {
        let target = VelocitySetpoint::default();
        assert_eq!(target.speed_step, SpeedStep::Stop);
        assert_eq!(target.direction, Direction::Forward);
    }

    #[test]
    fn test_new_speed_control() {
        let config = default_config();
        let control = Helper::new(config);
        assert_eq!(control.current, VelocitySetpoint::default());
        assert_eq!(control.target, VelocitySetpoint::default());
    }

    #[test]
    fn test_set_target() {
        let mut control = Helper::new(default_config());
        let new_target = VelocitySetpoint {
            speed_step: SpeedStep::Num(50),
            direction: Direction::Forward,
        };
        control.set_target(new_target);
        assert_eq!(control.target, new_target);
    }

    #[test]
    fn test_step_when_at_target_returns_max_delay() {
        let mut control = Helper::new(default_config());
        let (current, delay) = control.step();
        assert_eq!(current, VelocitySetpoint::default());
        assert_eq!(delay, Duration::MAX);
    }

    #[test]
    fn test_acceleration_with_rate() {
        let mut control = Helper::new(default_config());
        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(10),
            direction: Direction::Forward,
        });

        // First step should increment by 1
        let (current, delay) = control.step();
        assert_eq!(current.speed_step.idx(), 1);
        assert_eq!(current.direction, Direction::Forward);
        assert_eq!(delay, Duration::from_millis(7 * 10)); // accel_rate * loop_delay
    }

    #[test]
    fn test_acceleration_steps_gradually() {
        let mut control = Helper::new(default_config());
        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(5),
            direction: Forward,
        });

        let mut speeds = Vec::new();
        for _ in 0..10 {
            let (current, delay) = control.step();
            speeds.push(current.speed_step.idx());
            if delay == Duration::MAX {
                break;
            }
        }

        assert_eq!(speeds, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_acceleration_without_rate_jumps_immediately() {
        let mut control = Helper::new(config_no_ramp());
        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(50),
            direction: Direction::Forward,
        });

        let (current, _) = control.step();
        assert_eq!(current.speed_step.idx(), 50);
    }

    #[test]
    fn test_deceleration_with_rate() {
        let mut control = Helper::new(default_config());

        // Start at control 10
        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(10),
            direction: Direction::Forward,
        };
        control.target = control.current;

        // Set target to lower control
        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(5),
            direction: Direction::Forward,
        });

        let (current, delay) = control.step();
        assert_eq!(current.speed_step.idx(), 9);
        assert_eq!(delay, Duration::from_millis(7 * 10)); // decel_rate * loop_delay
    }

    #[test]
    fn test_deceleration_steps_gradually() {
        let mut control = Helper::new(default_config());

        // Start at control 5
        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(5),
            direction: Forward,
        };
        control.target = control.current;

        // Decelerate to stop
        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Stop,
            direction: Forward,
        });

        let mut speeds = Vec::new();
        loop {
            let (current, delay) = control.step();
            speeds.push(current.speed_step.idx());
            if delay == Duration::MAX {
                break;
            }
        }

        assert_eq!(speeds, vec![4, 3, 2, 1, 0]);
    }

    #[test]
    fn test_deceleration_without_rate_jumps_immediately() {
        let mut control = Helper::new(config_no_ramp());

        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(50),
            direction: Direction::Forward,
        };
        control.target = control.current;

        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Stop,
            direction: Direction::Forward,
        });

        let (current, _) = control.step();
        assert_eq!(current.speed_step.idx(), 0);
    }

    #[test]
    fn test_emergency_stop_immediate() {
        let mut control = Helper::new(default_config());

        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(50),
            direction: Direction::Forward,
        };
        control.target = control.current;

        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::EmergencyStop,
            direction: Direction::Forward,
        });

        let (current, _) = control.step();
        assert_eq!(current.speed_step, SpeedStep::EmergencyStop);
    }

    #[test]
    fn test_direction_change_decelerates_to_stop_first() {
        let mut control = Helper::new(default_config());

        // Start moving forward at control 3
        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(3),
            direction: Forward,
        };
        control.target = control.current;

        // Request reverse direction
        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(3),
            direction: Direction::Reverse,
        });

        // Should decelerate
        let (current, _) = control.step();
        assert_eq!(current.speed_step.idx(), 2);
        assert_eq!(current.direction, Forward);

        let (current, _) = control.step();
        assert_eq!(current.speed_step.idx(), 1);
        assert_eq!(current.direction, Forward);

        // Should reach stop and switch direction
        let (current, _) = control.step();
        assert_eq!(current.speed_step.idx(), SpeedStep::Stop.idx());
        assert_eq!(current.direction, Direction::Forward);

        let (current, _) = control.step();
        assert_eq!(current.speed_step.idx(), 1);
        assert_eq!(current.direction, Direction::Reverse);
    }

    #[test]
    fn test_direction_change_without_decel_rate_switches_immediately() {
        let mut control = Helper::new(config_no_ramp());

        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(50),
            direction: Forward,
        };
        control.target = control.current;

        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(30),
            direction: Direction::Reverse,
        });

        let (current, _) = control.step();
        assert_eq!(current.speed_step, SpeedStep::Num(30));
        assert_eq!(current.direction, Direction::Reverse);
    }

    #[test]
    fn test_direction_change_from_stop() {
        let mut control = Helper::new(default_config());

        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Stop,
            direction: Direction::Forward,
        };
        control.target = control.current;

        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(5),
            direction: Direction::Reverse,
        });

        let mut speeds = Vec::new();
        loop {
            let (current, delay) = control.step();
            speeds.push(current.speed_step.idx());
            if delay == Duration::MAX {
                break;
            }
        }

        assert_eq!(speeds, vec![1, 2, 3, 4, 5]);
        assert_eq!(control.current.direction, Direction::Reverse);
    }

    #[test]
    fn test_full_acceleration_cycle() {
        let mut control = Helper::new(Config {
            accel_rate: 5,
            decel_rate: 5,
            loop_delay: Duration::from_millis(10),
        });

        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Num(3),
            direction: Direction::Forward,
        });

        let mut total_time = Duration::ZERO;
        loop {
            let (_, delay) = control.step();
            if delay == Duration::MAX {
                break;
            }
            total_time += delay;
        }

        // Should take 3 steps, but the final step has an infinite delay so isnt counted.
        assert_eq!(total_time, Duration::from_millis(100));
    }

    #[test]
    fn test_full_deceleration_cycle() {
        let mut control = Helper::new(Config {
            accel_rate: 5,
            decel_rate: 5,
            loop_delay: Duration::from_millis(10),
        });

        control.current = VelocitySetpoint {
            speed_step: SpeedStep::Num(3),
            direction: Direction::Forward,
        };
        control.target = control.current;

        control.set_target(VelocitySetpoint {
            speed_step: SpeedStep::Stop,
            direction: Direction::Forward,
        });

        let mut total_time = Duration::ZERO;
        loop {
            let (_, delay) = control.step();
            if delay == Duration::MAX {
                break;
            }
            total_time += delay;
        }

        // Should take 3 steps, but the final step has an infinite delay so isnt counted.
        assert_eq!(total_time, Duration::from_millis(100));
    }
}
