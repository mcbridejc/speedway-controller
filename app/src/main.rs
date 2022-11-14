#![no_main]
#![no_std]

use core::sync::atomic::{AtomicBool, Ordering, AtomicU32};
use core::cell::RefCell;
use cortex_m;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use panic_halt as _;

use pwm::CurrentControl;
use stm32f0xx_hal as hal;

use touch::TouchConfig;
use touch::linear::HalfEndedLinear;
use touch::button::Button;

use crate::hal::pac;
use crate::hal::pac::interrupt;
use crate::hal::prelude::*;
use crate::hal::tsc::{Tsc};
use crate::hal::rcc::Rcc;

mod pwm;
mod serial;

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
            max_count: Some(hal::tsc::MaxCount::U8191),
            charge_transfer_high: None,
            charge_transfer_low: None,
        };
        Self {
            tsc: Tsc::tsc(tsc, rcc, Some(config)),
            max_count: 8191,
        }
    }

    pub fn sample<const N: usize>(&mut self, samples: [TscSample; N]) -> [u16; N] {
        // IMHO all HAL peripheral drivers should include a pub register block for just this kind of
        // extension, but they don't so steal our own.
        let regs = unsafe { pac::Peripherals::steal().TSC };

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
    TscSample { group: 6, sample: 2, channel: 3}, // Fwd 1
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

const MIN_STEP_FREQ: u32 = 8;
const MAX_STEP_FREQ: u32 = 900;
const ACCEL: u32 = 40;

static REVERSE_DIR: AtomicBool = AtomicBool::new(false);
static PWM: Mutex<RefCell<Option<CurrentControl>>> = Mutex::new(RefCell::new(None));
static TIME: AtomicU32 = AtomicU32::new(0);

struct StepTimer {
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

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut nvic = cp.NVIC;

    let mut flash = dp.FLASH;
    let mut rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut flash);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);


    // A library requiring a critical section to set a gpio AF register is bad and I just won't.
    let fake_cs = unsafe { cortex_m::interrupt::CriticalSection::new() };

    // Initialize touch pins
    let _rev1 = gpioa.pa0.into_alternate_af3(&fake_cs);
    let _rev2 = gpioa.pa4.into_alternate_af3(&fake_cs);
    let _rev3 = gpiob.pb0.into_alternate_af3(&fake_cs);
    let _stop = gpiob.pb11.into_alternate_af3(&fake_cs);
    let _fwd1 = gpiob.pb13.into_alternate_af3(&fake_cs);
    let _fwd2 = gpioa.pa5.into_alternate_af3(&fake_cs);
    let _fwd3 = gpiob.pb1.into_alternate_af3(&fake_cs);
    let _g1_cap = gpioa.pa1.into_alternate_af3(&fake_cs);
    let _g2_cap = gpioa.pa6.into_alternate_af3(&fake_cs);
    let _g3_cap = gpiob.pb2.into_alternate_af3(&fake_cs);
    let _g6_cap = gpiob.pb12.into_alternate_af3(&fake_cs);

    // Initialize timer pins
    let _g_in1 = gpiob.pb4.into_alternate_af1(&fake_cs);
    let _g_in2 = gpiob.pb5.into_alternate_af1(&fake_cs);
    let _a_in1 = gpioa.pa11.into_alternate_af2(&fake_cs);
    let _a_in2 = gpioa.pa10.into_alternate_af2(&fake_cs);
    let _b_in1 = gpioa.pa9.into_alternate_af2(&fake_cs);
    let _b_in2 = gpioa.pa8.into_alternate_af2(&fake_cs);

    let mut touch = TscWrapper::new(dp.TSC, &mut rcc);

    let mut pwm = pwm::CurrentControl::new(dp.TIM1, dp.TIM3, &mut rcc, 30.khz());
    // Fixed guard rail duty cycle for now.
    // We might want to scale this with input voltage to approximately constant current.
    pwm.set_guard_duty(12000);
    //pwm.set_phase_duty(Some(-8000), Some(8000));
    pwm.enable();
    cortex_m::interrupt::free(|cs| {
        PWM.borrow(cs).borrow_mut().replace(pwm);
    });

    let mut step_timer = StepTimer::new(dp.TIM2, &mut rcc, MIN_STEP_FREQ);

    unsafe {
        nvic.set_priority(pac::Interrupt::TIM2, 3);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
    }

    let mut syst = hal::timers::Timer::syst(cp.SYST, 100.hz(), &mut rcc);
    syst.listen(&hal::timers::Event::TimeOut);

    let tx_pin = gpiob.pb6.into_alternate_af0(&fake_cs);
    let rx_pin = gpiob.pb7.into_alternate_af0(&fake_cs);
    let uart = hal::serial::Serial::usart1(dp.USART1, (tx_pin, rx_pin), 115200.bps(), &mut rcc);
    serial::uart1::init(uart, 4);

    let mut rev_linear = HalfEndedLinear::new(Some(&TOUCH_CONFIG));
    let mut fwd_linear = HalfEndedLinear::new(Some(&TOUCH_CONFIG));
    let mut stop_button = Button::new(Some(&TOUCH_CONFIG));

    let mut target_velocity: i32 = 0;
    let mut current_velocity: i32 = 0;
    let mut next_time = 10;

    // step_timer.enable_irq();
    // step_timer.set_overflow_freq(2);

    loop {
        let time = TIME.load(Ordering::Relaxed);
        if time >= next_time {
            next_time += 4;

            let read1 = touch.sample(SAMPLE_GROUP1);

            cortex_m::asm::delay(1000);
            let read2 = touch.sample(SAMPLE_GROUP2);

            let mut rev_samples: [u16; 3] = [0; 3];
            rev_samples.copy_from_slice(&read1[0..3]);
            rev_linear.push(rev_samples);
            let rev_pos = rev_linear.pos();

            let mut fwd_samples: [u16; 3] = [0; 3];
            fwd_samples.copy_from_slice(&read2[0..3]);
            fwd_linear.push(fwd_samples);
            let fwd_pos = fwd_linear.pos();

            let stop_samples: [u16; 1] = [read1[3]];
            stop_button.push(stop_samples);

            target_velocity = if stop_button.active() {
                0
            } else if let Some(pos) = rev_pos {
                (touch::FULL_SCALE - pos) as i32 * MAX_STEP_FREQ as i32  / touch::FULL_SCALE as i32 + MIN_STEP_FREQ as i32
            } else if let Some(pos) = fwd_pos {
                -(pos as i32) * MAX_STEP_FREQ as i32  / touch::FULL_SCALE as i32 - MIN_STEP_FREQ as i32
            } else {
                target_velocity
            };

            if target_velocity > current_velocity {
                current_velocity += ACCEL as i32;
                if current_velocity > target_velocity {
                    current_velocity = target_velocity;
                }
            } else if target_velocity < current_velocity {
                current_velocity -= ACCEL as i32;
                if current_velocity < target_velocity {
                    current_velocity = target_velocity;
                }
            }

            cortex_m::interrupt::free(|cs| {
                let mut pwm_cell = PWM.borrow(cs).borrow_mut();
                let pwm = pwm_cell.as_mut().unwrap();

                if current_velocity.abs() < MIN_STEP_FREQ as i32 {
                    pwm.disable();
                    step_timer.disable_irq();
                } else {
                    REVERSE_DIR.store(current_velocity < 0, Ordering::Relaxed);
                    step_timer.set_overflow_freq(current_velocity.abs() as u32);
                    pwm.enable();
                    step_timer.enable_irq();
                }

                let magvel = current_velocity.abs();
                if magvel < 200 {
                    pwm.set_power(16000);
                } else if magvel < 500 {
                    pwm.set_power(18000);
                } else {
                    pwm.set_power(32767);
                }
            });
            // if time % 20 == 0 {
            //     let mut writer = serial::uart1::writer();
            //     //core::fmt::write(&mut writer, format_args!("REF: {} {} {}\r\n", rev_linear.reference[0], rev_linear.reference[1], rev_linear.reference[2]));

            //     // core::fmt::write(&mut writer, format_args!("READ {} {} {} {} {} {} {}\r\n",
            //     //     fwd_samples[0], fwd_samples[1], fwd_samples[2],
            //     //     rev_samples[0], rev_samples[1], rev_samples[2], stop_samples[0])).unwrap();
            //     core::fmt::write(&mut writer, format_args!("DELTA: {} {} {}\r\n", fwd_linear.deltas[0], fwd_linear.deltas[1], fwd_linear.deltas[2]));
            //     core::fmt::write(&mut writer, format_args!("POS: {} {} {}\r\n", fwd_linear.pos().unwrap_or(65535), rev_linear.pos().unwrap_or(65535), stop_button.active()));
            // }
        }

        //cortex_m::asm::wfi();

    }
}

#[exception]
fn SysTick() {
    let time = TIME.load(Ordering::Relaxed);
    TIME.store(time + 1, Ordering::Relaxed);
}

#[interrupt]
fn TIM2() {
    // Clear IRQ flags
    unsafe {
        let tim2 =  pac::Peripherals::steal().TIM2;
        tim2.sr.write(|w| w.bits(0));
    }
    let reverse = REVERSE_DIR.load(Ordering::Relaxed);

    // We don't need the mutex, because the IRQ is the highest prio accessor.
    // I hope this critical section creation optimizes away, but I'm not sure...
    let fake_cs = unsafe { cortex_m::interrupt::CriticalSection::new() };

    let mut pwm_cell = PWM.borrow(&fake_cs).borrow_mut();
    let pwm = pwm_cell.as_mut().unwrap();

    pwm.step(reverse);
}

