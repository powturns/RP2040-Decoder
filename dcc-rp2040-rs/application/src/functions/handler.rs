use crate::FunctionGroupOutputResources;
use dcc::cv::store::{Store, StoreExt};
use dcc::{FunctionGroup, FunctionGroupFlags};
use embassy_rp::Peri;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::pwm::{self, ChannelAPin, ChannelBPin, Pwm, PwmOutput, Slice};
use embedded_hal::pwm::SetDutyCycle;
use fixed::FixedU16;
use motor::Direction;

pub struct Config {
    /// Mapping from function group index to pin outputs in forward direction
    function_map_forward: [u32; 32],

    /// Mapping from function group index to pin outputs in reverse direction
    function_map_reverse: [u32; 32],
}

impl Config {
    pub fn new(function_map_forward: [u32; 32], function_map_reverse: [u32; 32]) -> Self {
        Self {
            function_map_forward,
            function_map_reverse,
        }
    }
}

pub fn new_handler<T: Store>(store: &T, resources: FunctionGroupOutputResources) -> Handler {
    let pwm_enabled_mask = store.pwm_enabled_mask();
    let function_map_forward: [u32; 32] = core::array::from_fn(|idx| {
        unwrap!(store.function_output_mask(idx as u8, Direction::Forward))
    });
    let function_map_reverse: [u32; 32] = core::array::from_fn(|idx| {
        unwrap!(store.function_output_mask(idx as u8, Direction::Reverse))
    });
    let config = Config::new(function_map_forward, function_map_reverse);

    let FunctionGroupOutputResources {
        gpio0,
        gpio1,
        gpio2,
        gpio3,
        gpio4,
        gpio5,
        aux0,
        aux1,
        aux2,
        aux3,
        pwm_slice0,
        pwm_slice1,
        pwm_slice2,
        pwm_slice4,
        pwm_slice5,
    } = resources;

    let (gpio0, gpio1) = build_slice_outputs(pwm_enabled_mask, pwm_slice0, gpio0, gpio1, store);
    let (gpio2, gpio3) = build_slice_outputs(pwm_enabled_mask, pwm_slice1, gpio2, gpio3, store);
    let (gpio4, gpio5) = build_slice_outputs(pwm_enabled_mask, pwm_slice2, gpio4, gpio5, store);
    let (aux0, aux1) = build_slice_outputs(pwm_enabled_mask, pwm_slice4, aux0, aux1, store);
    let (aux2, aux3) = build_slice_outputs(pwm_enabled_mask, pwm_slice5, aux2, aux3, store);

    let output_drivers = OutputDrivers {
        gpio0,
        gpio1,
        gpio2,
        gpio3,
        gpio4,
        gpio5,
        aux0,
        aux1,
        aux2,
        aux3,
    };

    Handler::new(config, output_drivers)
}

pub struct Handler {
    config: Config,

    direction: Direction,
    flags: FunctionGroupFlags,
    output_drivers: OutputDrivers,
}

impl Handler {
    fn new(config: Config, output_drivers: OutputDrivers) -> Self {
        Self {
            config,
            direction: Direction::Forward,
            flags: FunctionGroupFlags::empty(),
            output_drivers,
        }
    }

    pub fn handle(&mut self, function_group: FunctionGroup) {
        let old = self.flags;
        self.flags = function_group.union_flags(self.flags);

        if self.flags != old {
            self.update_outputs()
        }
    }

    pub fn set_direction(&mut self, direction: Direction) {
        if self.direction != direction {
            self.direction = direction;
            self.update_outputs();
        }
    }

    fn update_outputs(&mut self) {
        // map from function group index to pin outputs in the current direction
        let map = match self.direction {
            Direction::Forward => &self.config.function_map_forward,
            Direction::Reverse => &self.config.function_map_reverse,
        };

        // Accumulate which outputs should be on by OR-ing together all active function maps
        let mut output_mask = 0u32;
        for (func_idx, enabled_outputs) in map.iter().enumerate().take(32) {
            if (self.flags.bits() & 1u16 << func_idx) != 0 {
                output_mask |= enabled_outputs;
            }
        }

        debug!("updating fg output mask: {:010x}", output_mask);

        let outputs = [
            &mut self.output_drivers.gpio0,
            &mut self.output_drivers.gpio1,
            &mut self.output_drivers.gpio2,
            &mut self.output_drivers.gpio3,
            &mut self.output_drivers.gpio4,
            &mut self.output_drivers.gpio5,
            &mut self.output_drivers.aux0,
            &mut self.output_drivers.aux1,
            &mut self.output_drivers.aux2,
            &mut self.output_drivers.aux3,
        ];

        for output in outputs {
            if (output_mask & (1u32 << output.pin)) != 0 {
                output.set_on();
            } else {
                output.set_off();
            }
        }
    }
}

fn build_slice_outputs<TSlice, PA, PB, S>(
    pwm_enabled_mask: u32,
    slice: Peri<'static, TSlice>,
    pin_a: Peri<'static, PA>,
    pin_b: Peri<'static, PB>,
    store: &S,
) -> (OutputDriver, OutputDriver)
where
    TSlice: Slice,
    PA: ChannelAPin<TSlice> + Pin,
    PB: ChannelBPin<TSlice> + Pin,
    S: Store,
{
    let num_a = pin_a.pin();
    let num_b = pin_b.pin();
    let pwm_a = (pwm_enabled_mask & (1u32 << num_a)) != 0;
    let pwm_b = (pwm_enabled_mask & (1u32 << num_b)) != 0;
    let cfg = unwrap!(store.pwm_configuration(slice.number() as u8));

    if pwm_a || pwm_b {
        let mut pwm_cfg = pwm::Config::default();
        pwm_cfg.top = cfg.wrap;
        pwm_cfg.divider = FixedU16::from_num(cfg.divider as u16);

        match (pwm_a, pwm_b) {
            (true, true) => {
                let pwm = Pwm::new_output_ab(slice, pin_a, pin_b, pwm_cfg);
                let (out_a, out_b) = pwm.split();
                (
                    OutputDriver::from_pwm(num_a, cfg.a_level, unwrap!(out_a)),
                    OutputDriver::from_pwm(num_b, cfg.b_level, unwrap!(out_b)),
                )
            }
            (true, false) => {
                let pwm = Pwm::new_output_a(slice, pin_a, pwm_cfg);
                let (out_a, _) = pwm.split();
                (
                    OutputDriver::from_pwm(num_a, cfg.a_level, unwrap!(out_a)),
                    OutputDriver::from_gpio(num_b, Output::new(pin_b, Level::Low)),
                )
            }
            (false, true) => {
                let pwm = Pwm::new_output_b(slice, pin_b, pwm_cfg);
                let (_, out_b) = pwm.split();
                (
                    OutputDriver::from_gpio(num_a, Output::new(pin_a, Level::Low)),
                    OutputDriver::from_pwm(num_b, cfg.b_level, unwrap!(out_b)),
                )
            }
            (false, false) => unreachable!("handled in outer conditional"),
        }
    } else {
        (
            OutputDriver::from_gpio(num_a, Output::new(pin_a, Level::Low)),
            OutputDriver::from_gpio(num_b, Output::new(pin_b, Level::Low)),
        )
    }
}

struct OutputDrivers {
    gpio0: OutputDriver,
    gpio1: OutputDriver,
    gpio2: OutputDriver,
    gpio3: OutputDriver,
    gpio4: OutputDriver,
    gpio5: OutputDriver,
    aux0: OutputDriver,
    aux1: OutputDriver,
    aux2: OutputDriver,
    aux3: OutputDriver,
}

struct OutputDriver {
    pin: u8,
    pwm_level: u16,
    inner: OutputDriverImpl,
}

impl OutputDriver {
    fn from_gpio(pin: u8, output: Output<'static>) -> Self {
        Self {
            pin,
            pwm_level: 0,
            inner: OutputDriverImpl::Gpio(output),
        }
    }

    fn from_pwm(pin: u8, level: u16, pwm: PwmOutput<'static>) -> Self {
        Self {
            pin,
            pwm_level: level,
            inner: OutputDriverImpl::Pwm(pwm),
        }
    }

    pub fn set_on(&mut self) {
        self.inner.set_on(self.pwm_level);
    }

    pub fn set_off(&mut self) {
        self.inner.set_off();
    }
}

enum OutputDriverImpl {
    Gpio(Output<'static>),
    Pwm(PwmOutput<'static>),
}

impl OutputDriverImpl {
    fn set_on(&mut self, level: u16) {
        match self {
            OutputDriverImpl::Gpio(pin) => pin.set_high(),
            OutputDriverImpl::Pwm(pwm) => {
                let _ = pwm.set_duty_cycle(level);
            }
        }
    }

    fn set_off(&mut self) {
        match self {
            OutputDriverImpl::Gpio(pin) => pin.set_low(),
            OutputDriverImpl::Pwm(pwm) => {
                let _ = pwm.set_duty_cycle(0);
            }
        }
    }
}
