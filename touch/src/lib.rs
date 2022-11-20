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
    /// The number of counts of delta capacitance required to active the button
    pub detect_threshold: u16,
    /// The hysteresis in deactivation. Once active, counts must fall below `detect_threshold -
    /// detect_hysteresis` in order to deactivate.
    pub detect_hysteresis: u16,
    /// Number of samples to wait after initialization before starting calibration
    pub calibration_delay: u16,
    /// Number of sample to collect for reference level calibration
    pub calibration_samples: u16,
    /// Number of positive samples required to transition to Active state
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
