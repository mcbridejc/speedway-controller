#![allow(dead_code)]
#![cfg_attr(not(test), no_std)]

pub mod button;
pub mod linear;
pub mod tsc;

/// Enumeration of touch activity states

#[derive(Clone, Copy, Debug)]
pub enum TouchState {
    Startup(u16),
    Calibrate(u16),
    Idle(u16),
    Active,
}

/// Configuration structure for all touch inputs
#[derive(Clone, Copy, Debug)]
pub struct TouchConfig {
    pub detect_threshold: u16,
    pub detect_hysteresis: u16,
    pub calibration_delay: u16,
    pub calibration_samples: u16,
    /// Number of samples required to transition to touched
    pub debounce: u16,
}

impl TouchConfig {
    const fn default() -> Self {
        Self {
            detect_threshold: 100,
            detect_hysteresis: 5,
            calibration_delay: 10,
            calibration_samples: 16,
            debounce: 2,
        }
    }
}

pub const DEFAULT_TOUCH_CONFIG: TouchConfig = TouchConfig::default();
pub const FULL_SCALE: u16 = 1024;
const DEBOUNCE: u16 = 4;
