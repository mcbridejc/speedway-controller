#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;
use panic_halt as _;

use stm32f0xx_hal as hal;

use touch::TouchConfig;
use touch::linear::HalfEndedLinear;
use touch::button::Button;

use crate::hal::pac;
use crate::hal::prelude::*;
use crate::hal::tsc::{Tsc, TscPin};
use crate::hal::rcc::Rcc;

// mod touch_buttons;
// struct TouchUi<
//     Rev1Pin,
//     Rev2Pin,
//     Rev3Pin,
//     StopPin,
//     Fwd1Pin,
//     Fwd2Pin,
//     Fwd3Pin,
// > 
// {
//     rev1: Rev1Pin,
//     rev2: Rev2Pin,
//     rev3: Rev3Pin,
//     stop: StopPin,
//     fwd1: Fwd1Pin,
//     fwd2: Fwd2Pin,
//     fwd3: Fwd3Pin,
//     tsc: Tsc
// }

// impl <
//     Rev1Pin,
//     Rev2Pin,
//     Rev3Pin,
//     StopPin,
//     Fwd1Pin,
//     Fwd2Pin,
//     Fwd3Pin,
// > TouchUi<
//     Rev1Pin,
//     Rev2Pin,
//     Rev3Pin,
//     StopPin,
//     Fwd1Pin,
//     Fwd2Pin,
//     Fwd3Pin,
// > {
//     pub fn new(
//         rev1: Rev1Pin,
//         rev2: Rev2Pin,
//         rev3: Rev3Pin,
//         stop: StopPin,
//         fwd1: Fwd1Pin,
//         fwd2: Fwd2Pin,
//         fwd3: Fwd3Pin,
//         tsc: pac::TSC,
//         rcc: &mut Rcc,
//     ) -> Self {
        
//         Self {
//             rev1, rev2, rev3, stop, fwd1, fwd2, fwd3,
//             tsc: Tsc::tsc(tsc, rcc, None)
//         }
//     }

//     pub fn read_rev(&mut self) -> (u16, u16, u16) {

//     }
// }

#[derive(Clone, Copy, Debug)]
struct TscSample {
    group: u8,
    sample: u8,
    channel: u8,
}

struct TscWrapper {
    tsc: Tsc,
    max_count: u16,
}

impl TscWrapper {
    pub fn new(tsc: pac::TSC, rcc: &mut Rcc) -> Self {
        let config = hal::tsc::Config {
            clock_prescale: None,
            max_count: Some(hal::tsc::MaxCount::U16383),
            charge_transfer_high: None,
            charge_transfer_low: None,
        };
        Self {
            tsc: Tsc::tsc(tsc, rcc, Some(config)),
            max_count: 16383,
        }
    }

    pub fn sample<const N: usize>(&mut self, samples: [TscSample; N]) -> [u16; N] {
        // IMHO all HAL peripheral drivers should include a pub register block for just this kind of
        // extension, but they don't so steal our own.
        let mut regs = unsafe { pac::Peripherals::steal().TSC };

        let mut iogcsr: u32 = 0;
        let mut ioscr: u32 = 0;
        let mut ioccr: u32 = 0;

        for s in samples {
            // Enable the group
            iogcsr |= 1 << (s.group - 1);
            
            // Set the sample cap input
            ioscr |= 1 << (s.group - 1) * 4 + s.sample - 1;

            // Enable the input
            ioccr |= 1 << (s.group - 1) * 4 + s.channel - 1;
        }

        // Clear the enabled groups
        regs.iogcsr.write(|w| unsafe { w.bits(iogcsr) });
        regs.ioscr.write(|w| unsafe { w.bits(ioscr) });
        regs.ioccr.write(|w| unsafe { w.bits(ioccr) });

        self.tsc.acquire().ok();

        // Status bits indicate if the group completed successfully. Any group not completed
        // when MAX COUNT is reached will not be set.
        let group_status = regs.iogcsr.read().bits() >> 16;
        
        let mut result = [self.max_count + 1; N];
        for i in 0..samples.len() {
            let s = &samples[i];
            if group_status & (1 << (s.group - 1)) != 0 {
                result[i] = self.tsc.read_unchecked(s.group);
            }
        }

        result
    }
}

static SAMPLE_GROUP1: [TscSample; 4] = [
    TscSample { group: 1, sample: 2, channel: 1}, // Rev 1
    TscSample { group: 2, sample: 3, channel: 1}, // Rev 2
    TscSample { group: 3, sample: 4, channel: 2}, // Rev 3
    TscSample { group: 6, sample: 2, channel: 1}, // Stop
];

static SAMPLE_GROUP2: [TscSample; 3] = [
    TscSample { group: 6, sample: 2, channel: 1}, // Fwd 1
    TscSample { group: 2, sample: 3, channel: 2}, // Fwd 2
    TscSample { group: 3, sample: 4, channel: 3}, // Fwd 3
];

static TOUCH_CONFIG: TouchConfig = TouchConfig {
    detect_threshold: 100,
    detect_hysteresis: 5,
    calibration_delay: 10,
    calibration_samples: 10,
    debounce: 3,
};

fn read_all_touch(tsc: &mut TscWrapper) {



}

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH;
    let mut rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut flash);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // A library requiring a critical section to set a gpio AF register is bad and I just won't.
    let fake_cs = unsafe { cortex_m::interrupt::CriticalSection::new() };
    let rev1 = gpioa.pa0.into_alternate_af3(&fake_cs);
    let rev2 = gpioa.pa4.into_alternate_af3(&fake_cs);
    let rev3 = gpiob.pb0.into_alternate_af3(&fake_cs);
    let stop = gpiob.pb11.into_alternate_af3(&fake_cs);
    let fwd1 = gpiob.pb13.into_alternate_af3(&fake_cs);
    let fwd2 = gpioa.pa5.into_alternate_af3(&fake_cs);
    let fwd3 = gpiob.pb1.into_alternate_af3(&fake_cs);
    let g1_cap = gpioa.pa1.into_alternate_af3(&fake_cs);
    let g2_cap = gpioa.pa6.into_alternate_af3(&fake_cs);
    let g3_cap = gpiob.pb2.into_alternate_af3(&fake_cs);
    let g6_cap = gpiob.pb12.into_alternate_af3(&fake_cs);


    let mut touch = TscWrapper::new(dp.TSC, &mut rcc);



    // let sample_config1: [SamplingConfig; 4] = [
    //     rev1.group(), 
    // ];

    let mut rev_linear = HalfEndedLinear::new(Some(&TOUCH_CONFIG));
    let mut fwd_linear = HalfEndedLinear::new(Some(&TOUCH_CONFIG));
    let mut stop_button = Button::new(Some(&TOUCH_CONFIG));
    loop {
        let read1 = touch.sample(SAMPLE_GROUP1);
        let read2 = touch.sample(SAMPLE_GROUP2);
        
        let mut rev_samples: [u16; 3] = [0; 3];
        rev_samples.copy_from_slice(&read1[0..3]);
        rev_linear.push(rev_samples);

        let mut fwd_samples: [u16; 3] = [0; 3];
        fwd_samples.copy_from_slice(&read2[0..3]);
        fwd_linear.push(fwd_samples);
        
        let mut stop_samples: [u16; 1] = [read1[3]];
        stop_button.push(stop_samples);
    }
}
