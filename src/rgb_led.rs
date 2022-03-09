/// Module to encapsulate a PWM-driven 3 color LED
use embedded_hal::PwmPin;
use rp_pico::hal::{
    gpio::{
        bank0::{Gpio13, Gpio14, Gpio15},
        PullDown,
    },
    gpio::{Disabled, Pin},
    pwm::{Channel, FreeRunning, Pwm6, Pwm7, Slice, Slices, A, B},
};

enum Colors {
    Red,
    Green,
    Blue,
}

/// Struct to contain r, g and b values,
/// and a the three slices that feed them. There's
/// also a group of attributes used by the demo code
/// that should probably be in another struct, but
/// they use a lot of this struct so I just threw
/// them all together.
pub struct RgbLed {
    r: u16,
    g: u16,
    b: u16,

    color_active: Colors,
    scan_up: bool,
    brightness_b: u16,
    brightness_r: u16,
    brightness_g: u16,

    channel_r: Channel<Pwm6, FreeRunning, B>,
    channel_g: Channel<Pwm7, FreeRunning, A>,
    channel_b: Channel<Pwm7, FreeRunning, B>,
}

/// Create an RgbLed driver struct. It takes the pwm_slices
/// structure. This should be cleaned up to only take the
/// necessary slices, but right now it works, so it'll stay
/// the way it is.
impl RgbLed {
    pub fn new(
        pwm_slices: Slices,
        led_r_pin: Pin<Gpio13, Disabled<PullDown>>,
        led_g_pin: Pin<Gpio14, Disabled<PullDown>>,
        led_b_pin: Pin<Gpio15, Disabled<PullDown>>,
    ) -> RgbLed {
        let mut pwm_r: Slice<Pwm6, FreeRunning> = pwm_slices.pwm6;
        pwm_r.set_ph_correct();
        pwm_r.enable();
        let mut channel_r = pwm_r.channel_b;
        let led_r = led_r_pin;
        channel_r.output_to(led_r);
        channel_r.set_duty(0);

        let mut pwm_gb: Slice<Pwm7, FreeRunning> = pwm_slices.pwm7;
        pwm_gb.set_ph_correct();
        pwm_gb.enable();
        let mut channel_g = pwm_gb.channel_a;
        let led_g = led_g_pin;
        channel_g.output_to(led_g);
        channel_g.set_duty(0);

        let mut channel_b = pwm_gb.channel_b;
        let led_b = led_b_pin;
        channel_b.output_to(led_b);
        channel_b.set_duty(0);

        RgbLed {
            channel_r,
            channel_g,
            channel_b,
            color_active: Colors::Red,
            scan_up: true,
            brightness_r: 0,
            brightness_g: 0,
            brightness_b: 0,
            r: 0,
            g: 0,
            b: 0,
        }
    }
    /// Call to set the value on the Red channel of the LED
    pub fn set_r(&mut self, value: u16) {
        self.r = value;
        self.channel_r.set_duty(value);
    }
    /// Call to set the value on the Gree channel of the LED
    pub fn set_g(&mut self, value: u16) {
        self.g = value;
        self.channel_g.set_duty(value);
    }
    /// Call to set the value on the Blue channel of the LED
    pub fn set_b(&mut self, value: u16) {
        self.b = value;
        self.channel_b.set_duty(value);
    }

    // Demo. Call repeatedly to change colors
    // on RGB Led in waves. Not brilliant, but
    // kind of nice.
    pub fn color_demo(&mut self) {
        const STEP_SIZE: u16 = 1000;

        match self.color_active {
            Colors::Red => {
                self.set_g(if self.scan_up {
                    self.brightness_g
                } else {
                    65535 - self.brightness_g
                });
                if self.brightness_g as u32 + STEP_SIZE as u32 > 65535 {
                    self.brightness_g = 0;
                    self.color_active = Colors::Green;
                    self.scan_up = !self.scan_up;
                } else {
                    self.brightness_g += STEP_SIZE;
                };
            }
            Colors::Green => {
                self.set_b(if self.scan_up {
                    self.brightness_b
                } else {
                    65535 - self.brightness_b
                });
                if self.brightness_b as u32 + STEP_SIZE as u32 > 65535 {
                    self.brightness_b = 0;
                    self.color_active = Colors::Blue;
                    self.scan_up = !self.scan_up;
                } else {
                    self.brightness_b += STEP_SIZE;
                };
            }
            Colors::Blue => {
                self.set_r(if self.scan_up {
                    self.brightness_r
                } else {
                    65535 - self.brightness_r
                });
                if self.brightness_r as u32 + STEP_SIZE as u32 > 65535 {
                    self.brightness_r = 0;
                    self.color_active = Colors::Red;
                    self.scan_up = !self.scan_up;
                } else {
                    self.brightness_r += STEP_SIZE;
                };
            }
        }
    }
}
