Gauss Speedway Controller
=========================

Control software for the [Guass Speedway](https://github.com/mcbridejc/gauss-speedway).

This runs on an STM32F051 MCU, reads the capacitive touch sensors, and drives the three 
current outputs (two bipolar phases + one guard rail) accordingly. It also has a serial 
port to send data for debugging, or for whatever modifications might come up.

## Organization

There are two crates: 

- `app` is the main bin crate
- `touch` contains a driver for the TSC peripheral and processing for touch UI elements. 

I envision the touch crate could be a useful stand-alone and hope to do a more development on it and pull it out. 

TIM1 and TIM3 are used for generating the six 30kHz PWM signals (two per phase for controlling the
direction of the h-bridge driver), and the driver for this can be found in `app/src/pwm.rs`. The two
bipolar phases are driven like a stepper with sinusoidal microstepping. The TIM2 IRQ is used to
control the timing of motor steps.

USART1 on PB6/PB7 is used for reporting touch values, and available for further hacking as needed.
It's wired out to the 8-pin JST PH debug connector.

## Programming

To build: 
`cargo build --release`

To flash:

I use [cargo-flash](https://crates.io/crates/cargo-flash) and the programming command can be found
in `flash.sh`. 

Or use whatever programming tool you prefer.