use crate::pac;
use crate::hal::rcc::Rcc;

pub struct StepTimer {
    tim: pac::TIM2,
    clk_freq: u32,
}

impl StepTimer {
    pub fn new(tim: pac::TIM2, rcc: &mut Rcc, overflow_freq: u32) -> Self {
        let rccregs = unsafe { pac::Peripherals::steal().RCC };
        rccregs.apb1enr.modify(|_, w| w.tim2en().set_bit());

        // If pclk is prescaled from hclk, the frequency fed into the timers is doubled
        let clk_freq = if rcc.clocks.hclk().0 == rcc.clocks.pclk().0 {
            rcc.clocks.pclk().0
        } else {
            rcc.clocks.pclk().0 * 2
        };

        // start counter
        tim.cr1.modify(|_, w| {
            w.cen().set_bit()
            .arpe().set_bit()
        });

        let mut obj = Self {
            tim,
            clk_freq
        };
        obj.set_overflow_freq(overflow_freq);
        obj.tim.egr.write(|w| w.ug().set_bit());
        obj
    }

    pub fn enable_irq(&mut self) {
        self.tim.dier.write(|w| w.uie().set_bit());
    }

    pub fn disable_irq(&mut self) {
        self.tim.dier.write(|w| w.uie().clear_bit());
    }

    pub fn set_overflow_freq(&mut self, ovf_freq: u32) {
        let arr = self.clk_freq / ovf_freq;
        self.tim.arr.write(|w| w.arr().bits(arr));
    }
}