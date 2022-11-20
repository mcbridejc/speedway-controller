#![no_main]
#![no_std]

#[allow(dead_code)]
mod pwm;
mod serial;
mod step_timer;

use core::sync::atomic::{AtomicBool, Ordering, AtomicU32};
use core::cell::RefCell;
use cortex_m;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use panic_halt as _;

use pwm::CurrentControl;
use stm32f0xx_hal as hal;

use touch::{
    tsc::{Tsc, Channel, SampleConfig},
    TouchConfig,
    linear::HalfEndedLinear,
    button::Button,
};

use crate::hal::pac;
use crate::hal::pac::interrupt;
use crate::hal::prelude::*;

use step_timer::StepTimer;

// Configure the capacitive channels which will be read together
// During each acquisition, any number of groups can be active on selected
// channels. Each group has one pin dedicated to be used as the sampling
// capacitor, and up to three pins connected to pads to be measured. For this, it is
// setup so that all of the electrodes of a slider can be read simultaneously.
use Channel::*;
static SAMPLE_GROUP1: SampleConfig = SampleConfig::new()
    .sample(G1Ch2).channel(G1Ch1) // Rev 1
    .sample(G2Ch3).channel(G2Ch1) // Rev 2
    .sample(G3Ch4).channel(G3Ch2) // Rev 3
    .sample(G6Ch2).channel(G6Ch1); // Stop

static SAMPLE_GROUP2: SampleConfig = SampleConfig::new()
    .sample(G6Ch2).channel(G6Ch3) // Fwd 1
    .sample(G2Ch3).channel(G2Ch2) // Fwd 2
    .sample(G3Ch4).channel(G3Ch3); // Fwd 3

static TOUCH_CONFIG: TouchConfig = TouchConfig {
    detect_threshold: 200,
    detect_hysteresis: 50,
    calibration_delay: 10,
    calibration_samples: 10,
    debounce: 3,
};

/// Controls the frequency of the motor PWM outputs from TIM1/TIM3
const PWM_FREQUENCY: u32 = 30_000;
/// The minimum microstep frequency
/// At speeds below this, current is turned off
const MIN_STEP_FREQ: u32 = 8;
/// The maximum microstep frequency
const MAX_STEP_FREQ: u32 = 900;
/// The acceleration in steps/s per each 40ms period
const ACCEL: u32 = 40;
/// Duty cycle at slower speeds
const LOW_POWER_DUTY: i16 = 22000;
/// Duty cycle at high speeds
const HIGH_POWER_DUTY: i16 = 32767;
/// When velocity exceeds this threshold, switch from low to high power
const POWER_SPEED_THRESHOLD: i32 = 500;

static REVERSE_DIR: AtomicBool = AtomicBool::new(false);
static PWM: Mutex<RefCell<Option<CurrentControl>>> = Mutex::new(RefCell::new(None));
static TIME: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut nvic = cp.NVIC;

    let mut flash = dp.FLASH;
    let mut rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut flash);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // No critical section required.
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

    let mut touch = Tsc::new(None);

    let mut pwm = pwm::CurrentControl::new(dp.TIM1, dp.TIM3, &mut rcc, PWM_FREQUENCY.hz());

    // Fixed guard rail duty cycle for now.
    // We might want to scale this with input voltage to approximately constant current.
    pwm.set_guard_duty(12000);

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

    loop {
        let time = TIME.load(Ordering::Relaxed);
        if time >= next_time {
            next_time += 4;

            let mut fwd_samples = [0u16; 3];
            let mut rev_samples = [0u16; 3];
            let mut stop_samples = [0u16; 1];

            touch.acquire(&SAMPLE_GROUP1);
            rev_samples[0] = touch.read_group(1);
            rev_samples[1] = touch.read_group(2);
            rev_samples[2] = touch.read_group(3);
            stop_samples[0] = touch.read_group(6);

            // Some delay is needed to discharge the sampling capacitor
            cortex_m::asm::delay(1000);

            touch.acquire(&SAMPLE_GROUP2);
            fwd_samples[0] = touch.read_group(6);
            fwd_samples[1] = touch.read_group(2);
            fwd_samples[2] = touch.read_group(3);

            // Run touch sense processing
            rev_linear.push(rev_samples);
            fwd_linear.push(fwd_samples);
            stop_button.push(stop_samples);

            let fwd_pos = fwd_linear.pos();
            let rev_pos = rev_linear.pos();

            // Update target velocity if any buttons are pressed
            target_velocity = if stop_button.active() {
                0
            } else if let Some(pos) = rev_pos {
                (touch::FULL_SCALE - pos) as i32 * MAX_STEP_FREQ as i32  / touch::FULL_SCALE as i32 + MIN_STEP_FREQ as i32
            } else if let Some(pos) = fwd_pos {
                -(pos as i32) * MAX_STEP_FREQ as i32  / touch::FULL_SCALE as i32 - MIN_STEP_FREQ as i32
            } else {
                target_velocity
            };

            // Perform velocity slewing
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

                // Set output power based on speed. Empirically, less current is required at lower
                // speed
                let magvel = current_velocity.abs();
                if magvel < POWER_SPEED_THRESHOLD {
                    pwm.set_power(LOW_POWER_DUTY);
                } else {
                    pwm.set_power(HIGH_POWER_DUTY);
                }
            });

            // Periodically print the capacitive touch values out the serial port
            if time % 20 == 0 {
                let mut writer = serial::uart1::writer();
                core::fmt::write(&mut writer, format_args!("READ {} {} {} {} {} {} {}\r\n",
                    fwd_samples[0], fwd_samples[1], fwd_samples[2],
                    rev_samples[0], rev_samples[1], rev_samples[2],
                    stop_samples[0])
                ).unwrap();

                core::fmt::write(
                    &mut writer,
                    format_args!("Fwd: {} Rev: {} Stop: {}\r\n",
                        fwd_linear.pos().unwrap_or(65535),
                        rev_linear.pos().unwrap_or(65535),
                        stop_button.active()
                    )
                ).unwrap();
            }
        }
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
        let tim2 = &*pac::TIM2::ptr();
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
