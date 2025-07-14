#![no_std]
#![no_main]

// This must go FIRST so that all the other modules see its macros.
pub(crate) mod log;

mod decoder;

#[allow(unused_imports)]
#[cfg(feature = "probe-rs")]
use panic_probe as _;

#[allow(unused_imports)]
#[cfg(not(feature = "probe-rs"))]
use panic_reset as _;

#[allow(unused_imports)]
#[cfg(feature = "defmt-03")]
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{with_timeout, Duration, TimeoutError};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use crate::decoder::{PioDccDecoder, PioDccDecoderProgram};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn decoder(mut watchdog: Watchdog, mut decoder: PioDccDecoder<'static, PIO0, 0>) {
    info!("starting decoder loop");
    loop {
        let bit = with_timeout(Duration::from_secs(1), decoder.read()).await;
        
        match bit {
            Ok(bit) => {
                let bit = !bit;
                let bytes = bit.to_be_bytes();
                debug!("{} ({:b}) -> [{:08b}, {:08b}, {:08b}, {:08b}]", bit, bit, bytes[0], bytes[1], bytes[2], bytes[3])
            }
            Err(_) => {
                // trace!("timeout");
            }
        }
        
        // debug!("b{}", bit);
        watchdog.feed();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("dcc-rp2040-rs!");
    let p = embassy_rp::init(Default::default());

    // Override bootloader watchdog
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(8));

    let Pio {
        mut common, sm0, /*sm1,*/ ..
    } = Pio::new(p.PIO0, Irqs);

    // when logical high was longer than 87us, write 0 bit into buffer
    let prg = PioDccDecoderProgram::new(&mut common);
    let decoder1 = PioDccDecoder::new(&mut common, sm0, p.PIN_21, &prg);

    spawner.must_spawn(decoder(watchdog, decoder1));
}