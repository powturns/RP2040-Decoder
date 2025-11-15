mod speed;

use embassy_rp::gpio::{AnyPin, Input, Output, Pull};
use embedded_hal::digital::{InputPin, OutputPin};
use crate::MotorResources;

use embassy_rp::adc::{Adc, Async, Channel, Config, InterruptHandler};
use embassy_rp::{dma, Peri};
use embassy_rp::pwm::{Pwm, PwmOutput};
use embassy_rp::pwm;
use embedded_hal::pwm::SetDutyCycle;
use fixed::FixedU16;
use fixed::types::extra::U3;

pub struct MotorController<DMA: dma::Channel> {
    dir: Direction,
    fwd: DirectionControl,
    rev: DirectionControl,
    adc: Adc<'static, Async>,
    pub dma: Peri<'static, DMA>,
}

impl <DMA: dma::Channel> MotorController<DMA>{
    pub fn new(
        resources: MotorResources,
        adc: Adc<'static, Async>,
        dma: Peri<'static, DMA>,
    ) -> Self {

        let mut config = pwm::Config::default();
        config.compare_a = 0;
        config.compare_b = 0;
        //config.top = (_125M / (CV_ARRAY_FLASH[8] * 100 + 10000)) - 1;

        // pwm_set_clkdiv_int_frac(slice_num, CV_ARRAY_FLASH[173], 0);
        //config.divider =  CV_ARRAY_FLASH[173].into();

        let pwm = Pwm::new_output_ab(
            resources.pwm_slice,
            resources.rev_pin,
            resources.fwd_pin,
            config,
        );

        let (rev_out, fwd_out) = pwm.split();

        let fwd = DirectionControl::new(
            fwd_out.unwrap(),
            Channel::new_pin(resources.fwd_emf_in, Pull::None),
        );

        let rev = DirectionControl::new(
            rev_out.unwrap(),
            Channel::new_pin(resources.rev_emf_in, Pull::None),
        );

        Self {
            dir: Direction::Forward,
            fwd,
            rev,
            adc,
            dma,
        }
    }

    /// Measure the back emf of the motor.
    async fn measure(&mut self) -> Result<f32, ()> {
        // FIXME: restore the levels after the measurement?
        self.fwd.stop()?;
        self.rev.stop()?;

        // FIXME: use a configurable sample size
        const SAMPLE_CNT: usize = 100; //CV_ARRAY_FLASH[60]

        let buf = &mut [0; SAMPLE_CNT];

        match self.dir {
            Direction::Forward => self.fwd.measure_emf(&mut self.adc, buf, self.dma.reborrow()).await?,
            Direction::Reverse => self.rev.measure_emf(&mut self.adc, buf, self.dma.reborrow()).await?,
        }

        buf.sort_unstable();

        // outlier removal
        let l_side_arr_cutoff = 15; // CV_ARRAY_FLASH[62]
        let r_side_arr_cutoff = 15; // CV_ARRAY_FLASH[63]

        let filtered_cnt = SAMPLE_CNT - (l_side_arr_cutoff + r_side_arr_cutoff);

        let sum:u32 = buf.iter().skip(l_side_arr_cutoff).take(filtered_cnt).map(|&x| x as u32).sum();

        Ok(sum as f32/filtered_cnt as f32)
    }
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Direction {
    Forward,
    Reverse,
}

struct DirectionControl {
    motor_pwm: PwmOutput<'static>,
    emf: Channel<'static>
}


impl DirectionControl {

    pub fn new(
        motor_pwm: PwmOutput<'static>,
        emf: Channel<'static>,
    ) -> Self  {
        Self {
            motor_pwm,
           emf
        }
    }

    /// Measures the emf using the [adc] into the [buf].
    async fn measure_emf(
        &mut self,
        adc: &mut Adc<'_, Async>,
        buf: &mut [u16],
        dma: Peri<'_, impl dma::Channel>
    ) -> Result<(), ()> {
        adc.read_many(
            &mut self.emf,
            buf,
            0, // sample at the full 500 khz rate
            dma
        ).await.map_err(|_| ())?; // FIXME: proper error handling

        Ok(())
    }

    fn stop(&mut self) -> Result<(), ()> {
        self.motor_pwm.set_duty_cycle(0).map_err(|_| ()) // FIXME: proper error handling
    }
}