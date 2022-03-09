use defmt::info;
use embedded_error;
use embedded_hal::digital::v2::InputPin;
use rp_pico::hal::gpio::{
    self,
    bank0::{Gpio10, Gpio11, Gpio12, Gpio18, Gpio19, Gpio20, Gpio21},
    Floating, Input, Output, Pin, PullDown, PullUp, PushPull,
};
/// Contains routines to collect the switch and pot reading neatly
/// in one place
use rp_pico::Pins;

#[derive(PartialEq, Clone, Copy)]
pub enum Speed {
    Slow,
    Medium,
    Fast,
    Tramming,
}

enum Axis {
    X,
    Y,
    Z,
}

#[derive(Debug)]
pub enum ScanningError {
    SError(u8),
}

pub struct Xyz {
    p_x: Pin<Gpio10, Input<PullDown>>,
    p_y: Pin<Gpio11, Input<PullDown>>,
    p_z: Pin<Gpio12, Input<PullDown>>,
}

impl Xyz {
    pub fn new(
        p_x: Pin<Gpio10, Input<PullDown>>,
        p_y: Pin<Gpio11, Input<PullDown>>,
        p_z: Pin<Gpio12, Input<PullDown>>,
    ) -> Self {
        Xyz { p_x, p_y, p_z }
    }
}

pub struct Stepsize {
    s1: Pin<Gpio18, Input<PullUp>>,
    s2: Pin<Gpio19, Input<PullUp>>,
    s3: Pin<Gpio20, Input<PullUp>>,
    s4: Pin<Gpio21, Input<PullUp>>,
    current: Option<Speed>,
    next: Option<Speed>,
    count: u8,
}

/// How many of the same value do I need to see in succession
/// before accepting the output as valid?
const VALID_COUNT: u8 = 2;

impl Stepsize {
    pub fn new(
        s1: Pin<Gpio18, Input<PullUp>>,
        s2: Pin<Gpio19, Input<PullUp>>,
        s3: Pin<Gpio20, Input<PullUp>>,
        s4: Pin<Gpio21, Input<PullUp>>,
    ) -> Self {
        Stepsize {
            s1,
            s2,
            s3,
            s4,
            current: None,
            next: None,
            count: 0,
        }
    }

    // Convert a number from 1-4 to a Speed type
    pub fn axis_to_speed(axis: u8) -> Speed {
        match axis {
            0 => Speed::Tramming, // As wired!
            1 => Speed::Fast,
            2 => Speed::Slow,
            _ => Speed::Medium,
        }
    }

    /// Checks the three pins. Returns Ok(None) if no change,
    /// Ok(Axis) if there is a valid new axis and
    /// Error if zero or more than 1 pins is selected
    pub fn scan(&mut self) -> Option<Speed> {
        // Read the four inputs:
        let pins = [
            self.s1.is_low().unwrap(),
            self.s2.is_low().unwrap(),
            self.s3.is_low().unwrap(),
            self.s4.is_low().unwrap(),
        ];

        // info!("Pins: {:?}", pins);

        let mut axis: u8 = 0;
        let mut count = 0;
        // How many pins are high?
        for (i, set) in pins.iter().enumerate() {
            if *set {
                axis = (i % 256) as u8; // zero based and holds last pin that is high
                count += 1;
            }
        }

        if count == 1 {
            let now = Some(Stepsize::axis_to_speed(axis));
            // Has the pin changed?
            if self.current != now {
                // If the pin has changed, store the new pin in self.next
                // and start counting
                if now == self.next {
                    self.count += 1;

                    if self.count == VALID_COUNT {
                        self.current = self.next;
                    }
                } else {
                    self.next = now;
                    self.count = 0;
                }
            }
        } else {
            // If zero or more than 1 pins are high, output is None
            self.count = 0;
            self.current = None;
        }
        self.current
    }
}

// pub struct Inputs {
//     axis: Xyz,
//     speed: Stepsize,
//     pot: u16,
//     e_stop: bool,
//     master: bool,
// }

// impl Inputs {
//     pub fn new(
//         p_x: Pin<Gpio10, Input<PullDown>>,
//         p_y: Pin<Gpio11, Input<PullDown>>,
//         p_z: Pin<Gpio12, Input<PullDown>>,
//     ) -> Self {
//         Inputs {
//             axis: Axis::X,
//             speed: Speed::Slow,
//             pot: 32767,
//             e_stop: false,
//             master: false,
//         }
//     }
// }
