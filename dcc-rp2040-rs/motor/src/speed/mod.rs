use crate::speed::startup::ComputeResult;

mod accel;
mod table;
mod pid;
mod startup;


/// When the corrected measured voltage is above this value, the controller switches to PID mode.
const CONTROLLER_HANDOVER:f32 = 7.5f32;

struct Config {
    pid: pid::Config,
    startup: startup::Config,

    /// Offset to correct the measured voltage.
    adc_offset: f32,
}
struct Controller {
    config: Config,
    mode: Mode,
    pid: pid::Controller,
    startup: startup::Controller,
}

enum Mode {
    Startup,

    /// PID controller is active, with the specified feedforward value.
    PID(f32)
}

impl Controller {


    /// Resets the controller.
    ///
    /// Eg: when stopping / changing directions
    fn reset(&mut self) {
        self.startup.reset();
        self.pid.reset();
        self.mode = Mode::Startup;
    }

    fn compute(
        &mut self,
       measurement: f32,
       setpoint: u32,
       timestamp_ms: u64,
    ) -> Option<u16> {
        if setpoint == 0 {
            self.reset();
            return Some(0);
        }

        let measurement = measurement - self.config.adc_offset;

        match self.mode {
            Mode::Startup => match self.startup.compute(measurement) {
                ComputeResult::Output(out) => Some(out),
                ComputeResult::Handover(ff) => {
                    self.mode = Mode::PID(ff);
                    None
                }
            }
            Mode::PID(ff) => {
                self.pid.compute(
                    measurement,
                    setpoint as f32,
                    timestamp_ms,
                    ff,
                ).into()
            }
        }
    }
}
