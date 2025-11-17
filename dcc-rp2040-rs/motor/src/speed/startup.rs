use crate::speed::{CONTROLLER_HANDOVER};

const BASE_PWM_ARR_LEN:usize = 16;

pub struct Config {
    /// Maximum controller output (PWM top).
    ///
    /// FIXME
    /// (float) (_125M / (CV9 * 100 + 10000));
    pub output_max: u16,

    /// The PID feed forward factor.
    ///
    /// Used to scale the output when the motor started moving.
    pub pid_ff:f32,
}

/// A controller that quickly ramps up the output to overcome the friction of the motor.
///
/// This controller doesn't consider the target setpoint; its only goal is to get the motor moving.
pub struct Controller {
    config: Config,

    /// A buffer that stores the output value at which the handover target was reached.
    prev_start_output: [u16;BASE_PWM_ARR_LEN],
    buf_write_idx: usize,

    /// current output
    output: u16,
}

impl Controller {

    fn new(config: Config) -> Self {
        Self {
            config,
            prev_start_output: [0;BASE_PWM_ARR_LEN],
            buf_write_idx: 0,
            output: 0,
        }
    }

    /// Calculates the initial startup level by averaging the set base pwm values.
    fn get_initial_level(&self)-> u16 {
        let (sum, cnt) = self.prev_start_output.iter()
            .copied()
            .filter(|i| *i > 0)
            .fold((0_usize, 0_usize), |(sum, cnt), v| (sum+v as usize, cnt+1));

        if cnt != 0 {
            ((sum / cnt) * 2/3) as u16
        } else {
            0
        }
    }

    /// Reset the controller.
    ///
    /// Useful on direction changes or when stopping.
    pub fn reset(&mut self) {
        self.output = 0;
    }

    pub fn compute(
        &mut self,
        measurement: f32,
    ) -> ComputeResult {
        if self.output == 0 {
            self.output = self.get_initial_level()
        }

        if measurement < CONTROLLER_HANDOVER {
            let curr_output = self.output;

            // calculate the new output level for the next iteration
            self.output += self.config.output_max / 250;
            if (self.output > self.config.output_max) {
                // Try again with half value...
                self.output = self.get_initial_level() / 2;
            }

            ComputeResult::Output(curr_output)
        } else {
            // we've reached the handover target, save the current output in the buffer for next time
            self.prev_start_output[self.buf_write_idx] = self.output;
            self.buf_write_idx = (self.buf_write_idx+1)%self.prev_start_output.len();
            self.output = 0;

            ComputeResult::Handover(self.config.pid_ff * self.output as f32)
        }
    }
}

/// Result of a computation
pub enum ComputeResult {
    /// Set the output to the given value.
    Output(u16),

    /// Handover control to the PID controller, using the specified value as the feed forward value.
    Handover(f32),
}