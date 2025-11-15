#![no_std]
#![no_main]

// This must go FIRST so that all the other modules see its macros.
pub(crate) mod log;

mod cv;
mod transport;
mod timer;
mod motor;

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
use crate::transport::pio_decoder::PioDccDecoder;
use ::dcc::transport::{Decoder, Packet};
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, Peri, peripherals};
use embassy_rp::flash::{Async, FLASH_BASE, Flash};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIO0};
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::adc;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, TimeoutError, with_timeout, Instant};
use static_cell::StaticCell;
use dcc::handler::{Error, Handler, Op};
use dcc::is_recipient;
use dcc::transport::PacketError;
use crate::timer::InstantTimer;
use assign_resources::assign_resources;
use embassy_rp::adc::Adc;

// Provide FLASH_SIZE from build.rs-generated file.
include!(concat!(env!("OUT_DIR"), "/flash_consts.rs"));

type RawDecoder = PioDccDecoder<'static, PIO0, DMA_CH0, 0>;
type AppFlash = Flash<'static, FLASH, Async, FLASH_SIZE>;

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
        fwd_emf_in: PIN_29,
        rev_pin: PIN_22,
        rev_emf_in: PIN_28,
        pwm_slice: PWM_SLICE3,
        dma: DMA_CH2,
    }
}

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// TODO: if we start sending larger packets, consider using https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/zerocopy.rs
// FIXME: use a PubSubChannel instead of a channel so we can evict the oldest value if we cannot process them fast enough
static PACKET_CHANNEL: Channel<CriticalSectionRawMutex, Packet, 5> = Channel::new();

#[embassy_executor::task]
async fn decoder(/*mut watchdog: Watchdog,*/ mut decoder: Decoder<RawDecoder>) {
    info!("starting decoder loop");
    loop {
        let packet = decoder.read().await;

        info!("addr={} packet={:?}", packet.addr(), packet);

        PACKET_CHANNEL.try_send(packet);

        // FIXME: come up with a better watchdog
        // watchdog.feed();
    }
}

#[embassy_executor::task]
async fn handler(flash: AppFlash) {
    let cv_store =
        unwrap!(cv::FlashStore::new(flash, APP_FLASH_ORIGIN as u32 - FLASH_BASE as u32).await);

    let mut handler = Handler::new(
        InstantTimer::new(),
        cv_store,
    );

    loop {
        // TODO: if a packet hasn't been received within a heartbeat timeout
        //       we should execute a reset operation.
        //       We should probably do the same if we haven received a packet addressed to us
        //       in a while
        let packet = PACKET_CHANNEL.receive().await;

        match handler.handle(packet) {
            Ok(Some(op)) => match op {
                Op::AcknowledgeCv => {
                    todo!()
                }
            }
            Ok(None) => {
                /* noop */
            }
            Err(e) => {
                error!("error handling packet: {:?}", e);
            }
        }

    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    // core 0 executor
    let executor0 = EXECUTOR0.init(Executor::new());

    // Override bootloader watchdog
    // let mut watchdog = Watchdog::new(p.WATCHDOG);
    // watchdog.start(Duration::from_secs(8));



    let r = split_resources!(p);
    let dr = r.pio_decoder;

    let Pio {
        mut common,
        sm0, /*sm1,*/
        ..
    } = Pio::new(dr.pio, Irqs);
    let d = transport::pio_decoder(&mut common, sm0, dr.dcc_input, dr.dma);

    let fr = r.flash;
    let flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(fr.flash, fr.dma);

    let mut adc = Adc::new(p.ADC, Irqs, adc::Config::default());

    // start execution on core1
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            trace!("starting core1");
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.must_spawn(handler(flash));
            });
        },
    );

    executor0.run(|spawner| {
        spawner.must_spawn(decoder(/*watchdog,*/ d));
    });
}

