#![no_std]
mod log;

pub mod speed;
#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(any(test, feature = "debug"), derive(core::fmt::Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    Forward,
    Reverse,
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(any(test, feature = "debug"), derive(core::fmt::Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpeedStep {
    Stop,
    EmergencyStop,
    Num(u8),
}

impl SpeedStep {
    /// Index of the speed step.
    pub fn idx(&self) -> u8 {
        match self {
            SpeedStep::Stop | SpeedStep::EmergencyStop => 0,
            SpeedStep::Num(n) => *n,
        }
    }
}

/// A vector of speed and direction.
#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(any(test, feature = "debug"), derive(core::fmt::Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VelocitySetpoint {
    pub speed_step: SpeedStep,
    pub direction: Direction,
}

impl Default for VelocitySetpoint {
    fn default() -> Self {
        Self {
            speed_step: SpeedStep::Stop,
            direction: Direction::Forward,
        }
    }
}

impl VelocitySetpoint {
    pub fn new(speed_step: SpeedStep, direction: Direction) -> Self {
        Self {
            speed_step,
            direction,
        }
    }

    /// Checks if the current and other instance's index and direction are equivalent.
    fn equivalent(&self, other: &VelocitySetpoint) -> bool {
        self.speed_step.idx() == other.speed_step.idx() && self.direction == other.direction
    }
}
