use crate::{TouchConfig, TouchState, DEFAULT_TOUCH_CONFIG};

/// A button controls a capacitive pad which can be touched or not touched.
///
/// It is possible for a button to include multiple capacitance values, in which case the button
/// will activate when any of the inputs meet the requirements, and will deactivate when none of
/// then do.
///
/// A a TouchConfig struct can be (optionally) rovided to alter default parameters.
pub struct Button<'a, const N: usize> {
    pub reference: [u32; N],
    pub state: TouchState,
    pub config: &'a TouchConfig,
}

impl<'a, const N: usize> Button<'a, N> {
    /// Create a new button with optional configuration parameters
    pub fn new(config: Option<&'a TouchConfig>) -> Self {
        let config = config.unwrap_or(&DEFAULT_TOUCH_CONFIG);
        Self {
            reference: [0; N],
            state: TouchState::Startup(config.calibration_delay),

            config,
        }
    }

    /// Returns true if the button is in the Active state (i.e. is touched)
    pub fn active(&self) -> bool {
        match self.state {
            TouchState::Active => true,
            _ => false
        }
    }

    /// Process a new measurement for this touch button
    pub fn push(&mut self, measurements: [u16; N]) -> TouchState {
        let mut deltas = [0u16; N];

        for i in 0..N {
            let d = (self.reference[i] as i16) - (measurements[i] as i16);
            deltas[i] = if d < 0 { 0 } else { d as u16 };
        }

        self.state = match self.state {
            TouchState::Startup(counter) => {
                if counter == 0 {
                    TouchState::Calibrate(self.config.calibration_samples)
                } else {
                    TouchState::Startup(counter - 1)
                }
            },
            TouchState::Calibrate(counter) => {
                for i in 0..N {
                    self.reference[i] += measurements[i] as u32;
                }
                if counter == 1 {
                    for i in 0..N {
                        self.reference[i] /= self.config.calibration_samples as u32;
                    }
                    TouchState::Idle(self.config.debounce)
                } else {
                    TouchState::Calibrate(counter - 1)
                }
            },
            TouchState::Idle(counter) => {
                if deltas.iter().any(|x| *x >= self.config.detect_threshold) {
                    if counter == 1 {
                        TouchState::Active
                    } else {
                        TouchState::Idle(counter - 1)
                    }
                } else {
                    TouchState::Idle(self.config.debounce)
                }
            },
            TouchState::Active => {
                if deltas.iter().all(|x| *x < self.config.detect_threshold - self.config.detect_hysteresis) {
                    TouchState::Idle(self.config.debounce)
                } else {
                    TouchState::Active
                }
            }
        };

        self.state
    }

    /// Force a re-calibration of the reference capacitance
    pub fn force_calibrate(&mut self) {
        self.reference = [0; N];
        self.state = TouchState::Calibrate(self.config.calibration_samples);
    }
}

#[cfg(test)]
pub mod test {
    use super::*;

    #[test]
    pub fn test_button() {

        let config = &DEFAULT_TOUCH_CONFIG;
        let mut b = Button::new(Some(config));

        const REF: u16 = 1000;

        for _ in 0..config.calibration_delay + config.calibration_samples + 1 {
            b.push([REF]);
            assert!(!b.active());
        }

        // "Touch" it for enough samples to get through debounce
        for _ in 0..DEFAULT_TOUCH_CONFIG.debounce {
            assert!(!b.active());
            b.push([REF - config.detect_threshold - 1]);
        }

        // Now it should be active
        assert!(b.active(), "Not active after debounce");

        // Down to hysteresis value, it should remain active
        b.push([REF - config.detect_threshold + config.detect_hysteresis - 1]);
        assert!(b.active(), "Became inactive too soon despite hysteresis");

        b.push([REF]);
        assert!(!b.active(), "Didn't deactivate");

    }

    pub fn test_button_negative() {
        let config = &DEFAULT_TOUCH_CONFIG;
        let mut b = Button::new(Some(config));

        const REF: u16 = 100;

        for _ in 0..config.calibration_delay + config.calibration_samples + 1 {
            b.push([REF]);
            assert!(!b.active());
        }

        for _ in 0..config.debounce + 2 {
            assert!(!b.active());
            b.push([REF - 30]);
        }

        assert!(!b.active());
    }

}