use crate::hal;
use crate::hal::pac;
use crate::hal::time::Hertz;


// TODO: Would like to build this table and make NSTEPS a const generic, but need to get const trig functions working first.
// const fn sin_table_u16<const N: usize>() -> [u16; N] {
//     let mut i = 0;
//     let lut = [0; N];
//     while i < N {
//         let x = (2.0 * core::f32::consts::PI * i as f32 / N as f32).radians();
//         let y = const_trig::sin(x, None);
//         lut[i] = (32767.0 * y) as i16;
//     }
//     lut
// }

pub struct BipolarMicrostepper {
    scaled_lut: [u16; 4],
}

impl BipolarMicrostepper {
    // NSTEPS must be divisible by 4
    const NSTEPS: usize = 16;
    // partial lookup table for sin function; takes advantage of symmetry (N must be odd)
    const LUT: [u16; Self::NSTEPS/4] = [12539, 23169, 30272, 32767];

    pub fn new() -> Self {
        let mut scaled_lut = [0; 4];
        
        for i in 0..Self::LUT.len() {
            scaled_lut[i] = Self::LUT[i];
        }

        Self {
            scaled_lut
        }
    }

    pub fn set_scale(&mut self, full_scale: u16) {
        for i in 0..Self::LUT.len() {
            self.scaled_lut[i] = ((Self::LUT[i] as i32 * full_scale as i32) / 32768) as u16;
        }
    }

    pub fn scale(&self) -> u16 {
        self.scaled_lut[Self::NSTEPS/4 - 1]
    }

    fn lookup(&self, idx: usize) -> i32 {
        let negate = idx >= Self::NSTEPS / 2;
        let mod_index = idx % (Self::NSTEPS / 2);

        // In order to shrink the table, we take advantage of some symmetry.
        // This works as long as NSTEPS % 4 == 0.
        // First, the second half of the sine function is the negative of the first.
        // Second, the first sample is always 0, so we don't store it.
        // Finally, after the peak, there's another mirroring.
        // Does this result in smaller flash? For small tables, maybe not. The increased code size 
        // could outweigh the benefit? I haven't checked. 

        let mut lut_value = if mod_index == 0 {
            0
        } else if mod_index <= Self::NSTEPS / 4 {
            self.scaled_lut[mod_index - 1] as i32
        } else {
            self.scaled_lut[Self::NSTEPS / 2 - mod_index - 1] as i32
        };
        if negate {
            lut_value = -lut_value;
        }

        lut_value
    }

    pub fn get(&self, step: usize) -> (i32, i32) {
        assert!(step < Self::NSTEPS);
        let a = self.lookup(step);
        let b = self.lookup((step + Self::NSTEPS / 4) % Self::NSTEPS);
        (a, b)
    }

    pub fn nsteps(&self) -> usize {
        Self::NSTEPS
    }
}

pub struct CurrentControl {
    tim1: pac::TIM1,
    tim3: pac::TIM3,
    stepper: BipolarMicrostepper,
    stepper_pos: u16,
    enabled: bool,
    power: i16,
}

impl CurrentControl {
    pub fn new<T>(tim1: pac::TIM1, tim3: pac::TIM3, rcc: &mut hal::rcc::Rcc, frequency: T) -> Self 
    where T: Into<Hertz>
    {
        let mut obj = Self {
            tim1: tim1,
            tim3: tim3,
            stepper: BipolarMicrostepper::new(),
            stepper_pos: 0,
            enabled: false,
            power: 32767,
        };
        let full_scale = obj.configure(frequency, rcc.clocks);
        // Update the lookup table to the appropraite scale based on timer config
        obj.stepper.set_scale(full_scale);
        obj
    }

    pub fn configure<T>(&mut self, frequency: T, clocks: hal::rcc::Clocks) -> u16
    where T: Into<Hertz>
    {
        let rcc = unsafe { pac::Peripherals::steal().RCC };
        
        // Enable and reset both timers
        rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
        rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());

        let frequency = frequency.into().0;
        // If pclk is prescaled from hclk, the frequency fed into the timers is doubled
        let tclk = if clocks.hclk().0 == clocks.pclk().0 {
            clocks.pclk().0
        } else {
            clocks.pclk().0 * 2
        };
        let ticks = tclk / frequency;

        let psc = ((ticks - 1) / (1 << 16)) as u16;
        self.tim1.psc.write(|w| w.psc().bits(psc));
        self.tim3.psc.write(|w| w.psc().bits(psc));

        let arr = (ticks / (psc + 1) as u32) as u16;
        self.tim1.arr.write(|w| w.arr().bits(arr));
        self.tim3.arr.write(|w| w.arr().bits(arr));

        // TIM1 has all output channel enabled
        self.tim1.ccmr1_output().modify(|_, w| {
            w.oc1m().pwm_mode1()
            .oc2m().pwm_mode1()
            .oc1pe().enabled()
            .oc2pe().enabled()
        });
        self.tim1.ccmr2_output().modify(|_, w| {
            w.oc3m().pwm_mode1()
            .oc4m().pwm_mode1()
            .oc3pe().enabled()
            .oc4pe().enabled()
        });
        self.tim1.ccer.modify(|_, w| {
            w.cc1e().set_bit()
            .cc2e().set_bit()
            .cc3e().set_bit()
            .cc4e().set_bit()
        });
        self.tim1.bdtr.modify(|_, w| {
            w.moe().enabled()

        });

        // TIM3 uses only CH1/2
        self.tim3.ccmr1_output().modify(|_, w| {
            w.oc1m().pwm_mode1()
            .oc2m().pwm_mode1()
            .oc1pe().enabled()
            .oc2pe().enabled()
        });
        self.tim3.ccer.modify(|_, w| {
            w.cc1e().set_bit()
            .cc2e().set_bit()
        });

        // start counter
        self.tim1.cr1.modify(|_, w| w.cen().set_bit());
        self.tim3.cr1.modify(|_, w| w.cen().set_bit());

        arr
    }

    pub fn set_power(&mut self, power: i16) {
        self.power = power;
    }

    /// Set the duty cycle on the two motor phases
    /// 
    /// Duty cycle has range [-32767, 32767].
    /// 
    /// None can be passed and no action will be taken for that channel.
    pub fn set_phase_duty(&mut self, a: Option<i16>, b: Option<i16>) {
        if let Some(a) = a {
            let scaled_duty = (a as i32 * self.tim1.arr.read().arr().bits() as i32 / 32768) as i32;
            self.set_phase_a_raw(scaled_duty)
        }
        if let Some(b) = b {
            let scaled_duty = (b as i32 * self.tim1.arr.read().arr().bits() as i32 / 32768) as i32;
            self.set_phase_b_raw(scaled_duty);
        }
    }

    /// Set the duty cycle on the guard rail PWM output
    /// 
    /// Duty cycle has range [-32767, 32767]
    pub fn set_guard_duty(&mut self, duty: i16) {
        let scaled_duty = (duty as i32 * self.tim1.arr.read().arr().bits() as i32 / 32768) as i32;
        self.set_guard_raw(scaled_duty)
    }

    /// Advance the microstepper position 1 step
    /// 
    /// If reverse, "advance" backwards
    pub fn step(&mut self, reverse: bool) {
        if reverse {
            self.stepper_pos -= 1;
        } else {
            self.stepper_pos += 1;
        }
        self.stepper_pos = self.stepper_pos % self.stepper.nsteps() as u16;

        let (a, b) = self.stepper.get(self.stepper_pos as usize);

        self.set_phase_a_raw((a * self.power as i32) / 32768);
        self.set_phase_b_raw((b * self.power as i32) / 32768);
    }

    /// Enable current outputs
    pub fn enable(&mut self) {
        self.tim1.bdtr.modify(|_, w| {
            w.moe().enabled() 
        });
        self.tim3.ccer.modify(|_, w| {
            w.cc1e().set_bit()
            .cc2e().set_bit()
        });
    }

    /// Disablea all outputs
    pub fn disable(&mut self) {
        self.tim1.bdtr.modify(|_, w| {
            w.moe().disabled_idle() 
        });
        self.tim3.ccer.modify(|_, w| {
            w.cc1e().clear_bit()
            .cc2e().clear_bit()
        });
    }

    fn set_phase_a_raw(&mut self, value: i32) {
        if value >= 0 {
            self.tim1.ccr3.write(|w| w.ccr().bits(0));
            self.tim1.ccr4.write(|w| w.ccr().bits(value.abs() as u16));
        } else {
            self.tim1.ccr3.write(|w| w.ccr().bits(value.abs() as u16));
            self.tim1.ccr4.write(|w| w.ccr().bits(0));
        }
    }
    
    fn set_phase_b_raw(&mut self, value: i32) {
        if value >= 0 {
            self.tim1.ccr1.write(|w| w.ccr().bits(0));
            self.tim1.ccr2.write(|w| w.ccr().bits(value.abs() as u16));
        } else {
            self.tim1.ccr1.write(|w| w.ccr().bits(value.abs() as u16));
            self.tim1.ccr2.write(|w| w.ccr().bits(0));
        }
    }

    fn set_guard_raw(&mut self, value: i32) {
        if value >= 0 {
            self.tim3.ccr1.write(|w| w.ccr().bits(0));
            self.tim3.ccr2.write(|w| w.ccr().bits(value.abs() as u16));
        } else {
            self.tim3.ccr1.write(|w| w.ccr().bits(value.abs() as u16));
            self.tim3.ccr2.write(|w| w.ccr().bits(0));
        }
    }
}   