use defmt::export::usize;
use embassy_rp::gpio::{AnyPin, Input, Output, Pull};
use embedded_hal::digital::{InputPin, OutputPin};
use crate::MotorResources;

use embassy_rp::adc::{Adc, Async, Channel, Config as AdcConfig, InterruptHandler};
use embassy_rp::{dma, Peri};
use embassy_rp::pwm::{Pwm, PwmOutput};
use embassy_rp::pwm;
use embedded_hal::pwm::SetDutyCycle;
use math::filtered_mean;

const ADC_CALIBRATION_ITERATIONS:usize = 8192;

pub struct Config {
    pub pwm_max_output: u16,
    pub pwm_clk_divider: fixed::FixedU16<fixed::types::extra::U4>,
}

pub struct MotorController<DMA: dma::Channel> {
    dir: Direction,
    fwd: DirectionControl,
    rev: DirectionControl,
    adc: Adc<'static, Async>,
    pub dma: Peri<'static, DMA>,
}

impl <DMA: dma::Channel> MotorController<DMA>{
    pub fn new(
        config: Config,
        resources: MotorResources,
        adc: Adc<'static, Async>,
        dma: Peri<'static, DMA>,
    ) -> Self {

        let mut pwm_config = pwm::Config::default();
        pwm_config.compare_a = 0;
        pwm_config.compare_b = 0;
        pwm_config.top = config.pwm_max_output; // FIXME: c code subtracts 1
        pwm_config.divider = config.pwm_clk_divider;

        let pwm = Pwm::new_output_ab(
            resources.pwm_slice,
            resources.rev_pin,
            resources.fwd_pin,
            pwm_config,
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

    async fn adc_offset_adjustment(&mut self) -> Result<u16, ()> {
        let buf = &mut [0; ADC_CALIBRATION_ITERATIONS];

        info!("Measuring ADC offset in reverse direction. n={}", buf.len());
        let _ = &mut self.fwd.measure_emf(&mut self.adc, buf, self.dma.reborrow()).await?;
        let offset_avg_fwd = filtered_mean(buf, 2).unwrap_or(0);
        info!("offset_avg_fwd={}", offset_avg_fwd);

        info!("Measuring ADC offset in forward direction. n={}", buf.len());
        let _ = &mut self.rev.measure_emf(&mut self.adc, buf, self.dma.reborrow()).await?;
        let offset_avg_rev = filtered_mean(buf, 2).unwrap_or(0);
        info!("offset_avg_rev={}", offset_avg_rev);

        Ok((offset_avg_fwd as u32 + offset_avg_rev as u32 / 2) as u16)
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