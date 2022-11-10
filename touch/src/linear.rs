
enum State {
    Calibrate,
    Idle,
    Active,
}

pub struct HalfEndedConfig {
    detect_threshold: u16,
    detect_hysteresis: u16,
    calibration_delay: u16,
    calibration_samples: u16,
}

impl HalfEndedConfig {
    const fn default() -> Self {
        Self {
            detect_threshold: 100,
            detect_hysteresis: 5,
            calibration_delay: 10,
            calibration_samples: 16,
        }
    }
}

const DEFAULT_HALF_ENDED_CONFIG: HalfEndedConfig = HalfEndedConfig::default();
pub const FULL_SCALE: u16 = 1024;
const DEBOUNCE: u16 = 4;

pub fn half_ended_pos(deltas: &[u16], detect_threshold: u16) -> Option<u16> {
    let mut delta_high: u16 = 0;
    let mut delta_mid: u16 = 0;
    let mut delta_low: u16 = 0;
    let mut index_high: u16 = 0;
    let mut index_mid: u16 = 0;

    // Find the three highest deltas and sort them.
    for i in 0..deltas.len() {
        // Saturate at zero because negative delta means noise.
        let delta = deltas[i];
        
        if delta > delta_high {
            delta_low = delta_mid;
            delta_mid = delta_high;
            index_mid = index_high;
            delta_high = delta;
            index_high = i as u16;
        } else if delta > delta_mid {
            delta_low = delta_mid;
            delta_mid = delta;
            index_mid = i as u16;
        } else if delta > delta_low {
            delta_low = delta;
        }
    }

    // Require two electrodes to have significant signal to calculate a position
    if delta_mid < detect_threshold {
        return None;
    }

    // The two electrodes must be neighbors
    if (index_mid as i16 - index_high as i16).abs() != 1 && 
       !((index_high == 0 || index_mid == 0) && 
       (index_high == deltas.len() as u16 - 1 || index_mid == deltas.len() as u16 - 1)) {
        return None;
    }

    // The position on the linear is normalized so that 0 between the first and second electrode,
    // and 1 is between the n-1 and n-2 electrodes, on the right. It is assumed that the
    // measurements provided are ordered left-to-right, with the exception that the first sensor is
    // be found on both ends.
    
    // Compute the center of the segment with the mid capacitance (delta_high)
    let segment_size = FULL_SCALE as i16 / (deltas.len() as i16 - 1);
    let center = if index_high == 0 {
        if index_mid == 1 {
            // The left is end is active
            -segment_size / 2
        } else if index_mid == deltas.len() as u16 - 1 {
            // The right end is active
            (deltas.len() as i16 - 1) * segment_size - segment_size / 2
        } else {
            // Inconsistent
            // Here, the largest delta is the end pads, but the second largest is not a neighbnor. 
            return None
        }
    } else {
        index_high as i16 * segment_size - segment_size / 2
    };

    // Now that we found the center point of the most active pad, find an adjustment based on the other two
    // Adjustment is negative if the mid pad is left of the high pad
    let negative_sign = index_high == 0 && index_mid == 1 || index_high > index_mid;
    let adjustment = (segment_size as u32 * (delta_high - delta_low) as u32) / (delta_high + delta_mid + delta_low) as u32;
    let mut pos = if negative_sign {
        center - adjustment as i16
    } else {
        center + adjustment as i16
    };
    if pos < 0 {
        pos = 0
    } else if pos > FULL_SCALE as i16 - 1 {
        pos = FULL_SCALE as i16 - 1;
    }

    Some(pos as u16)
}


/// Implement a linear touch input with "half-end" configuation, where the 
/// first electrode and last electrodes are half-size, and both connected to 
/// channel one. The electrode array looks like:
/// 
/// ---------------------------
/// | 1 |  2  | ... |  N  | 1 |
/// ---------------------------
pub struct HalfEndedLinear<'a, const N: usize> {
    reference: [u32; N], /// Store reference offset, and accumulations during calibration
    state: State,
    counter: i16, // Used for debounce, and other state
    config: &'a HalfEndedConfig,
}

impl<'a, const N: usize> HalfEndedLinear<'a, N> {
    

    pub fn new(config: Option<&'a HalfEndedConfig>) -> Self {
        let config = config.unwrap_or(&DEFAULT_HALF_ENDED_CONFIG);
        Self { 
            reference: [0; N],
            state: State::Calibrate,
            counter: -(config.calibration_delay as i16),
            config,
        }
    }

    pub fn calc_pos(&mut self, deltas: [u16; N]) -> Option<u16> {
        half_ended_pos(&deltas, self.config.detect_threshold)
    }

    /// Input a new sample for all electrodes in the linear array
    /// 
    /// Returns the latest positions if a touch is detected, otherwise None
    pub fn push(&mut self, measurements: [u16; N]) -> Option<u16> {

        let mut deltas = [0u16; N];
        for i in 0..N {
            deltas[i] = (self.reference[i] as u16).saturating_sub(measurements[i]);
        }

        match self.state {
            State::Calibrate => {
                self.counter += 1;
                if self.counter > 0 && self.counter < self.config.calibration_samples as i16 {
                    for i in 0..N {
                        self.reference[i] += measurements[i] as u32;
                    }    
                } else if self.counter == self.config.calibration_samples as i16 {
                    for i in 0..N {
                        self.reference[i] /= self.config.calibration_samples as u32;
                    }
                    self.state = State::Idle;
                }
                None
            },
            State::Idle => {
                if deltas.iter().any(|x| *x > self.config.detect_threshold) {
                    self.counter += 1;
                    if self.counter > DEBOUNCE as i16 {
                        self.state = State::Active;
                        self.counter = 0;
                        self.calc_pos(deltas)
                    } else {
                        None
                    }
                } else {
                    self.counter = 0;
                    None
                }
            },
            State::Active => {
                let pos = self.calc_pos(deltas);
                if pos.is_none() {
                    self.state = State::Idle;
                }
                pos
            },
        }
    }

    /// Begin a calibration
    pub fn force_calibrate(&mut self) {

    }
}

#[cfg(test)]
pub mod test {
    use super::*;

    #[test]
    fn test_half_ended_pos() {
        
        let detect_threshold = 15;
        
        let pos = half_ended_pos(&[0, 20, 20], detect_threshold);
        assert_eq!(pos, Some(512));

        let pos = half_ended_pos(&[0, detect_threshold - 1, 20], detect_threshold);
        assert!(pos.is_none(), "Failed to return none for signal below detect threshold");

        let pos = half_ended_pos(&[100, 100, 0], detect_threshold);
        assert_eq!(pos, Some(0));

        let pos = half_ended_pos(&[120, 60, 10], detect_threshold);
        assert_eq!(pos, Some(0));

        let pos = half_ended_pos(&[100, 200, 100], detect_threshold);
        assert_eq!(pos, Some(128));

        let pos = half_ended_pos(&[100, 200, 80], detect_threshold);
        assert_eq!(pos, Some(95));

        let pos = half_ended_pos(&[200, 0, 20], detect_threshold);
        assert_eq!(pos, Some(1023));

        let pos = half_ended_pos(&[200, 10, 200], detect_threshold);
        assert_eq!(pos, Some(1005));
    }
}