
use crate::{TouchConfig, TouchState};
use crate::FULL_SCALE;
use crate::button::Button;

pub fn half_ended_pos(deltas: &[u16], detect_threshold: u16) -> Option<u16> {
    let mut delta_high: i32 = 0;
    let mut delta_mid: i32 = 0;
    let mut delta_low: i32 = 0;
    let mut index_high: u32 = 0;
    let mut index_mid: u32 = 0;
    let mut index_low: u32 = 0;

    // Find the three highest deltas and sort them.
    for i in 0..deltas.len() {
        // Saturate at zero because negative delta means noise.
        let delta = deltas[i] as i32;

        if delta >= delta_high {
            delta_low = delta_mid;
            delta_mid = delta_high;
            delta_high = delta;
            index_low = index_mid;
            index_mid = index_high;
            index_high = i as u32;
        } else if delta >= delta_mid {
            delta_low = delta_mid;
            delta_mid = delta;
            index_low = index_mid;
            index_mid = i as u32;
        } else if delta >= delta_low {
            delta_low = delta;
            index_low = i as u32;
        }
    }

    // Require two electrodes to have significant signal to calculate a position
    if delta_mid < detect_threshold as i32 {
        return None;
    }

    // The two electrodes must be neighbors
    if (index_mid as i16 - index_high as i16).abs() != 1 &&
       !((index_high == 0 || index_mid == 0) &&
       (index_high == deltas.len() as u32 - 1 || index_mid == deltas.len() as u32 - 1)) {
        return None;
    }

    // The position on the linear is normalized so that 0 between the first and second electrode,
    // and 1 is between the n-1 and n-2 electrodes, on the right. It is assumed that the
    // measurements provided are ordered left-to-right, with the exception that the first sensor is
    // be found on both ends.

    let segment_size: i32 = FULL_SCALE as i32 / (deltas.len() as i32 - 1);
    let x0: i32  = if index_low != 0 {
        segment_size * (index_low as i32) - segment_size / 2
    } else {
        // It's a half-end
        if index_high == 1 {
            -(segment_size / 2)
        } else {
            FULL_SCALE as i32 + segment_size / 2
        }
    };
    let x1: i32 = if index_mid != 0 {
        segment_size * (index_mid as i32) - segment_size / 2
    } else {
        if index_high == 1 {
            -(segment_size / 2)
        } else {
            FULL_SCALE as i32 + segment_size / 2
        }
    };

    let x2: i32 = if index_high != 0 {
        segment_size * (index_high as i32) - segment_size / 2
    } else {
        if index_mid == 1 {
            -(segment_size / 2)
        } else {
            FULL_SCALE as i32 + segment_size / 2
        }
    };

    // println!("Index: {} {} {}", index_low, index_mid, index_high);
    // println!("Delta: {} {} {}", delta_low, delta_mid, delta_high);
    // println!("x0: {}, x1: {}, x2: {}", x0, x1, x2);
    // xn are 10 bits, deltas are (at most) 14 bits. i32 is safe from overflow.
    let mut center = (x0 * delta_low + x1 * delta_mid + x2 * delta_high) / (delta_low + delta_mid + delta_high);

    if center < 0 {
        center = 0;
    } else if center > FULL_SCALE as i32 - 1 {
        center = FULL_SCALE as i32 - 1;
    }

    Some(center as u16)
}


/// Implement a linear touch input with "half-end" configuation, where the
/// first electrode and last electrodes are half-size, and both connected to
/// channel one. The electrode array looks like:
///
/// ---------------------------
/// | 1 |  2  | ... |  N  | 1 |
/// ---------------------------
pub struct HalfEndedLinear<'a, const N: usize> {
    pub button: Button<'a, N>,
    pub deltas: [u16; N],
}

impl<'a, const N: usize> HalfEndedLinear<'a, N> {
    pub fn new(config: Option<&'a TouchConfig>) -> Self {
        Self {
            button: Button::new(config),
            deltas: [0; N],
        }
    }

    pub fn pos(&mut self) -> Option<u16> {
        if self.active() {
            half_ended_pos(&self.deltas, self.button.config.detect_threshold)
        } else {
            None
        }
    }

    pub fn active(&self) -> bool {
        self.button.active()
    }
    /// Input a new sample for all electrodes in the linear array
    ///
    /// Returns the latest positions if a touch is detected, otherwise None
    pub fn push(&mut self, measurements: [u16; N]) -> Option<u16> {
        for i in 0..N {
            self.deltas[i] = (self.button.reference[i] as u16).saturating_sub(measurements[i]);
        }

        let state = self.button.push(measurements);

        match state {
            TouchState::Active => {
                self.pos()
            },
            _ => None
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
        assert_eq!(pos, Some(256));

        let pos = half_ended_pos(&[100, 200, 80], detect_threshold);
        assert_eq!(pos, Some(229));

        let pos = half_ended_pos(&[200, 0, 20], detect_threshold);
        assert_eq!(pos, Some(1023));

        let pos = half_ended_pos(&[200, 10, 200], detect_threshold);
        assert_eq!(pos, Some(1005));

        let pos = half_ended_pos(&[5200, 0, 5800], detect_threshold);
        assert_eq!(pos, Some(1010));


        let pos = half_ended_pos(&[0, 3900, 5800], detect_threshold);
        assert_eq!(pos, Some(562));
    }
}