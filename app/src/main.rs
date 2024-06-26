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
    detect_threshold: 1000,
    detect_hysteresis: 40,
    calibration_delay: 10,
    calibration_samples: 10,
    debounce: 3,
};

/// Controls the frequency of the motor PWM outputs from TIM1/TIM3
const PWM_FREQUENCY: u32 = 35_000;
/// The minimum microstep frequency
/// At speeds below this, current is turned off
const MIN_STEP_FREQ: u32 = 8;
/// The maximum microstep frequency
const MAX_STEP_FREQ: u32 = 1200;
/// The acceleration in steps/s per each 40ms period
const ACCEL: u32 = 100;
/// When velocity exceeds this threshold, switch from low to high power
const POWER_SPEED_THRESHOLD: i32 = 500;

static REVERSE_DIR: AtomicBool = AtomicBool::new(false);
static PWM: Mutex<RefCell<Option<CurrentControl>>> = Mutex::new(RefCell::new(None));
static TIME: AtomicU32 = AtomicU32::new(0);


/// Desired current for guard rails (amps)
const IGUARD: f32 = 0.8;
/// Desired peak current for phase traces (amps)
const IDRIVE_LOW: f32 = 0.7;
const IDRIVE_HIGH: f32 = 1.0;
// Resistance of guard including trace and driver (ohms)
const RGUARD: f32 = 0.8 + 1.4;
/// Resistance of phase including trace and driver (ohms)
const RDRIVE: f32 = 5.25 + 1.4;
/// Scales counts to volts for ADC measureuemt
const VSCALE: f32 = 3.22e-3;


fn get_guard_dutycycle(vin: u16) -> i16 {
    // Precompute the numerator at compile time, so we don't need float routines
    const NUM: u32 = (32768.0 * IGUARD * RGUARD / VSCALE) as u32;
    let duty: u32 = NUM / vin as u32;

    if duty > 32767 {
        32767
    } else {
        duty as i16
    }
}

// Get the drive power based on input voltage and current velocity
fn get_phase_power(vin: u16, vel: i32) -> i16 {
    // Precompute the numerator at compile time, so we don't need float routines
    const NUM_HIGH: u32 = (32768.0 * IDRIVE_HIGH * RDRIVE / VSCALE) as u32;
    const NUM_LOW: u32 = (32768.0 * IDRIVE_LOW * RDRIVE / VSCALE) as u32;
    let duty: u32 = if vel < POWER_SPEED_THRESHOLD {
        NUM_LOW / vin as u32
    } else {
        NUM_HIGH / vin as u32
    };

    if duty > 32767 {
        32767
    } else {
        duty as i16
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

    // Get input voltage sense pin
    let mut vin_sense = gpioa.pa3.into_analog(&fake_cs);
    let mut adc = hal::adc::Adc::new(dp.ADC, &mut rcc);

    let touch_options = touch::tsc::Config {
        clock_prescale: None,
        charge_transfer_high: Some(touch::tsc::ChargeDischargeTime::C2),
        charge_transfer_low: Some(touch::tsc::ChargeDischargeTime::C2),
        max_count: Some(touch::tsc::MaxCount::U8191),
    };
    let mut touch = Tsc::new(Some(touch_options));

    let mut pwm = pwm::CurrentControl::new(dp.TIM1, dp.TIM3, &mut rcc, PWM_FREQUENCY.hz());

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
    rev_linear.set_scale_calibration([436, 191, 875]);
    let mut fwd_linear = HalfEndedLinear::new(Some(&TOUCH_CONFIG));
    fwd_linear.set_scale_calibration([196, 232, 187]);
    let mut stop_button = Button::new(Some(&TOUCH_CONFIG));

    let mut target_velocity: i32 = 0;
    let mut current_velocity: i32 = 0;
    let mut next_time = 10;
    let mut next_tx = 10;

    let mut vin_filt: u32 = 0;

    loop {
        let time = TIME.load(Ordering::Relaxed);
        if time >= next_time {
            next_time += 4;

            let mut fwd_samples = [0u16; 3];
            let mut rev_samples = [0u16; 3];
            let mut stop_samples = [0u16; 1];

            touch.acquire(&SAMPLE_GROUP1);
            rev_samples[0] = touch.read_group(1);
            rev_samples[1] = touch.read_group(3);
            rev_samples[2] = touch.read_group(2);
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

            let vsense_read: u16 = adc.read(&mut vin_sense).unwrap();
            vin_filt = (9 * vin_filt + vsense_read as u32) / 10;

            // Update target velocity if any buttons are pressed
            target_velocity = if stop_button.active() {
                0
            } else if let Some(pos) = rev_pos {
                pos as i32 * MAX_STEP_FREQ as i32  / touch::FULL_SCALE as i32 + MIN_STEP_FREQ as i32
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
                // Scale guard duty cycle based on input voltage
                pwm.set_guard_duty(get_guard_dutycycle(vin_filt as u16));
                // Scale phase duty cycle based on input voltage and speed
                pwm.set_power(get_phase_power(vin_filt as u16, current_velocity.abs()));

                if current_velocity.abs() < MIN_STEP_FREQ as i32 {
                    pwm.disable();
                    step_timer.disable_irq();
                } else {
                    REVERSE_DIR.store(current_velocity < 0, Ordering::Relaxed);
                    step_timer.set_overflow_freq(current_velocity.abs() as u32);
                    pwm.enable();
                    step_timer.enable_irq();
                }

            });

            // Periodically print the capacitive touch values out the serial port
            if time > next_tx {
                next_tx = next_tx + 20;
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

                core::fmt::write(
                    &mut writer,
                    format_args!("VIN: {}\r\n", vin_filt),
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
