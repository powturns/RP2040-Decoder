#![no_std]
#![no_main]

// This must go FIRST so that all the other modules see its macros.
pub(crate) mod log;

mod functions;
mod motor;
mod store;
mod timer;
mod timers;
mod transport;

#[allow(unused_imports)]
#[cfg(feature = "probe-rs")]
use panic_probe as _;

#[allow(unused_imports)]
#[cfg(not(feature = "probe-rs"))]
use panic_reset as _;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use defmt_rtt as _;

// use crate::transport::Decoder;
use crate::motor::{Command, Controller as MotorController, RpMotorController};
use crate::timer::InstantTimer;
use crate::transport::pio_decoder::PioDccDecoder;
use ::dcc::transport::{Decoder, packet::Packet};
use ::motor::speed::{
    Config as SpeedConfig, Controller as SpeedController, PidConfig, StartupConfig, accel,
};
use assign_resources::assign_resources;
use dcc::cv::store::{StoreExt, ensure_populated};
use dcc::handler::{Handler, Op};
use embassy_executor::{Spawner};
use embassy_rp::adc;
use embassy_rp::adc::Adc;
use embassy_rp::flash::{Async, FLASH_BASE, Flash};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIO0};
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::{PubSubBehavior, PubSubChannel};
use embassy_time::Duration;
use functions::handler::Handler as FunctionGroupHandler;

const FLASH_SIZE: usize = 2 * 1024 * 1024;

// Provide FLASH_SIZE from build.rs-generated file.
include!(concat!(env!("OUT_DIR"), "/flash_consts.rs"));

type RawDecoder = PioDccDecoder<'static, PIO0, DMA_CH0, 0>;
type AppFlash = Flash<'static, FLASH, Async, FLASH_SIZE>;

type Packethandler = Handler<InstantTimer, store::Flash<'static, FLASH, FLASH_SIZE>>;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

// Pinout
// GPIO 21: DCC input

// GPIO 23: Motor forward
// GPIO 29: ADC input for Back-EMF voltage (forward)
// GPIO 22: Motor reverse
// GPIO 28: ADC input for Back-EMF voltage (reverse)

assign_resources! {
    flash: FlashResources {
        flash: FLASH,
        dma: DMA_CH1,
    }
    pio_decoder: PioDecoderResources {
        pio: PIO0,
        dcc_input: PIN_21,
        dma: DMA_CH0,
    }
    motor: MotorResources {
        fwd_pin: PIN_23,
        fwd_emf_in: PIN_28,
        rev_pin: PIN_22,
        rev_emf_in: PIN_29,
        pwm_slice: PWM_SLICE3,
    }
    motor_dma: MotorDma {
        dma: DMA_CH2,
    }
    function_group: FunctionGroupOutputResources {
        gpio0: PIN_0,
        gpio1: PIN_1,
        gpio2: PIN_2,
        gpio3: PIN_3,
        gpio4: PIN_4,
        gpio5: PIN_5,

        aux0: PIN_24,
        aux1: PIN_25,
        aux2: PIN_26,
        aux3: PIN_27,

        pwm_slice0: PWM_SLICE0,
        pwm_slice1: PWM_SLICE1,
        pwm_slice2: PWM_SLICE2,
        pwm_slice4: PWM_SLICE4,
        pwm_slice5: PWM_SLICE5,
    }
}

// static mut CORE1_STACK: Stack<4096> = Stack::new();
// static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// TODO: if we start sending larger packets, consider using https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/zerocopy.rs
// FIXME: use a PubSubChannel instead of a channel so we can evict the oldest value if we cannot process them fast enough
static PACKET_CHANNEL: PubSubChannel<ThreadModeRawMutex, Packet, 5, 1, 0> = PubSubChannel::new();

static MOTOR_CHANNEL: Channel<ThreadModeRawMutex, motor::Command, 5> = Channel::new();

#[embassy_executor::task]
async fn packet_decoder(/*mut watchdog: Watchdog,*/ mut decoder: Decoder<RawDecoder>) {
    trace!("starting decoder loop");

    loop {
        let packet = decoder.read().await;

        if cfg!(feature = "verbose-transport") {
            debug!("addr={} packet={:?}", packet.addr(), packet);
        }

        PACKET_CHANNEL.publish_immediate(packet);
    }
}

#[embassy_executor::task]
async fn packet_handler(mut handler: Packethandler, mut fg_handler: FunctionGroupHandler) {
    trace!("starting packet handler loop");

    let mut receiver = unwrap!(PACKET_CHANNEL.subscriber());

    let mut last_command: Option<Op> = None;

    loop {
        // TODO: if a packet hasn't been received within a heartbeat timeout
        //       we should execute a reset operation.
        //       We should probably do the same if we haven received a packet addressed to us
        //       in a while
        let packet = receiver.next_message_pure().await;

        if cfg!(feature = "verbose-transport") {
            trace!("handling packet: {:?}", packet);
        }

        let op = handler.handle(&packet);

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("dcc-rp2040-rs startup!");
    let p = embassy_rp::init(Default::default());

    // Override bootloader watchdog
    // let mut watchdog = Watchdog::new(p.WATCHDOG);
    // watchdog.start(Duration::from_secs(8));

    let r = split_resources!(p);
    let dr = r.pio_decoder;

    let decoder = {
        let Pio {
            mut common,
            sm0, /*sm1,*/
            ..
        } = Pio::new(dr.pio, Irqs);

        transport::pio_decoder(&mut common, sm0, dr.dcc_input, dr.dma)
    };

    let flash = {
        let fr = r.flash;
        AppFlash::new(fr.flash, fr.dma)
    };

    let mut cv_store =
        unwrap!(store::Flash::new(flash, CV_FLASH_ORIGIN as u32 - FLASH_BASE as u32).await);

    // let MAX_MEASUREMENT = 550;
    // let vhigh = (MAX_MEASUREMENT/16) as u8;
    // let vmid = (vhigh as f32 * 0.6) as u8;
    // info!("vhigh={}, vmid={}", vhigh, vmid);
    // cv_store.write_byte(5, vhigh);
    // cv_store.write_byte(6, vmid);

    // let PWM_FREQUENCY = 25_000_u32;
    // let pwm = (PWM_FREQUENCY-10_000)/100;
    // info!("pwm={}", pwm);
    // cv_store.write_byte(9, pwm as u8);
    //
    // // feed forward factor to ~0.5
    // cv_store.write_byte(47, 200);

    unwrap!(ensure_populated(&mut cv_store));

    if cfg!(feature = "defmt") {
        info!("decoder addr = {}", cv_store.addr());
    }

    let motor_pwm_frequency = unwrap!(cv_store.motor_pwm_frequency());
    debug!("motor_pwm_frequency={}", motor_pwm_frequency);
    let output_max = (embassy_rp::clocks::clk_sys_freq() / motor_pwm_frequency) as u16;

    let mut motor_controller = {
        let adc = Adc::new(p.ADC, Irqs, adc::Config::default());

        let config = motor::Config {
            pwm_max_output: output_max,
            pwm_clk_divider: fixed::FixedU16::from_num(unwrap!(cv_store.motor_pwm_divider()) as u16),
            emf_measurement_delay: unwrap!(unwrap!(cv_store.emf_measurement_delay()).try_into()),
        };

        debug!("motor::Config={:?}", config);

        RpMotorController::new(config, r.motor, adc, r.motor_dma.dma)
    };

    let adc_offset = match unwrap!(cv_store.emf_adc_offset()) {
        Some(o) => o,
        None => {
            // calculate the offset, and store it in the store
            debug!("calculating adc offset");

            match motor_controller.measure_adc_offset().await {
                Ok(measured_offset) => {
                    if measured_offset >= u8::MAX as u16 {
                        error!("adc offset is too large: {}, truncating", measured_offset);
                    }
                    let measured_offset = measured_offset.clamp(0, (u8::MAX - 1) as u16) as u8;

                    info!("calculated adc offset: {}", measured_offset);

                    unwrap!(cv_store.write_emf_adc_offset(measured_offset));
                    measured_offset
                }
                Err(e) => {
                    error!("unable to measure adc offset: {:?}. Defaulting to 0", e);
                    0
                }
            }
        }
    };

    let speed_table = {
        use ::motor::speed::table;
        let config = table::Config {
            v_start: unwrap!(cv_store.v_start()),
            v_mid: unwrap!(cv_store.v_mid()),
            v_high: unwrap!(cv_store.v_high()),
        };

        debug!("table::Config={:?}", config);

        table::build(config)
    };

    let speed_control = {
        let config = SpeedConfig::new(
            PidConfig {
                sample_time: unwrap!(cv_store.pid_sample_time()),
                filter_tc: unwrap!(cv_store.pid_filter_tc()),
                ki: unwrap!(cv_store.pid_ki()),
                kd: unwrap!(cv_store.pid_kd()),
                output_max,
                kp_gain_range1_end: unwrap!(cv_store.pid_kp_gain_range1_end()),
                kp_y0: unwrap!(cv_store.pid_kp_y0()),
                kp_y1: unwrap!(cv_store.pid_kp_y1()),
                kp_y2: unwrap!(cv_store.pid_kp_y2()),
                max_setpoint: *unwrap!(speed_table.last()) as f32,
            },
            StartupConfig {
                output_max,
                pid_ff: unwrap!(cv_store.pid_k_ff()),
            },
            adc_offset as f32,
        );

        debug!("motor::speed::Config={:?}", config);

        unwrap!(SpeedController::new(config))
    };

    let accel_helper = {
        let config = accel::Config {
            accel_rate: unwrap!(cv_store.acceleration_rate()),
            decel_rate: unwrap!(cv_store.deceleration_rate()),
            loop_delay: unwrap!(cv_store.speed_step_period()),
        };

        debug!("accel::Config={:?}", config);

        accel::Helper::new(config)
    };

    let pid_sample_time = unwrap!(cv_store.pid_sample_time());
    let fg_handler = functions::handler::new_handler(&cv_store, r.function_group);

    let ph = Handler::new(InstantTimer::new(), cv_store);

    // start execution on core1
    // FIXME: bring back this core if we cant do it all on one.
    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         trace!("starting core1");
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(|spawner| {
    //             spawner.must_spawn(packet_handler(ph));
    //         });
    //     },
    // );

    spawner.must_spawn(packet_decoder(/*watchdog,*/ decoder));
    spawner.must_spawn(packet_handler(ph, fg_handler));
    motor::handler::spawn(
        spawner,
        pid_sample_time
            .try_into()
            .unwrap_or(Duration::from_millis(7)),
        motor_controller,
        speed_control,
        accel_helper,
        speed_table,
    );
}
