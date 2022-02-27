//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::{
    delay::Delay,
    interrupt::{free, Mutex},
};
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
// use embedded_hal::Pwm::Channel;
use embedded_hal::PwmPin;
// use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint;
use panic_halt as _;
use rp_pico as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        self, bank0::Gpio25, bank0::Gpio8, bank0::Gpio9, Floating, Input, Output, Pin, PushPull,
    },
    pac::{self, interrupt},
    pwm::{Channel, FreeRunning, Pwm6, Slice, Slices, B},
    sio::Sio,
    watchdog::Watchdog,
};

type LEDPinType = Pin<Gpio25, Output<PushPull>>;
// static G_LED_PIN: Mutex<RefCell<Option<LEDPinType>>> = Mutex::new(RefCell::new(None));

type CountPinType = Pin<Gpio8, Input<Floating>>;
type DirPinType = Pin<Gpio9, Input<Floating>>;
static G_COUNTPIN: Mutex<RefCell<Option<CountPinType>>> = Mutex::new(RefCell::new(None));
static G_DIRPIN: Mutex<RefCell<Option<DirPinType>>> = Mutex::new(RefCell::new(None));
static G_COUNT: Mutex<RefCell<Option<i32>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // let delay = Delay::new(core.SYST, clocks);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let alarm0 = timer.alarm_0().unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_r = pins.gpio13;
    let mut led_g = pins.gpio14;
    let mut led_b = pins.gpio15;

    // Setup PWMs
    // There are 8 PWM slices, each with A and B channels
    // r is on slice 6, channel A, g is on slice 7 channel A
    // b is on slice 7 channel B

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut rgb_leds = RgbLed::new(pwm_slices);

    let pwm_r = &mut pwm_slices.pwm6;
    pwm_r.set_ph_correct();
    pwm_r.enable();
    let channel_r = &mut pwm_r.channel_b;
    channel_r.output_to(led_r);
    channel_r.set_duty(0);

    // let pwm_g = &mut pwm_slices.pwm7;
    // pwm_g.set_ph_correct();
    // pwm_g.enable();
    // let channel_g = &mut pwm_g.channel_a;
    // channel_g.output_to(led_g);
    // channel_g.set_duty(0);

    // // let pwm_b = &mut pwm_slices.pwm7;
    // // pwm_b.set_ph_correct();
    // // pwm_b.enable();
    // let channel_b = &mut pwm_g.channel_b;
    // channel_b.output_to(led_b);
    // channel_b.set_duty(0);

    let led_pin: LEDPinType = pins.led.into_push_pull_output();
    let count_pin: CountPinType = pins.gpio8.into_floating_input();
    let count = 0;
    let dir_pin: DirPinType = pins.gpio9.into_floating_input();
    count_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    free(|cs| {
        G_DIRPIN.borrow(cs).replace(Some(dir_pin));
        G_COUNTPIN.borrow(cs).replace(Some(count_pin));
        G_COUNT.borrow(cs).replace(Some(count));
    });
    let mut old: i32 = 0;
    let mut pwm_r: u16 = 0;
    let mut pwm_g: u16 = 0;
    let mut pwm_b: u16 = 0;
    let mut g_active = false;
    loop {
        free(|cs| {
            if let Some(count) = G_COUNT.borrow(cs).borrow_mut().deref_mut() {
                if old != *count {
                    old = *count;
                    info!("Count: {}", old);
                }
            };
        });
        delay.delay_ms(50);

        // let step_size = 1000;

        // if g_active {
        //     channel_g.set_duty(65535 - pwm_g);
        //     if pwm_g as u32 + step_size as u32 > 65535 {
        //         pwm_g = 0;
        //         g_active = false;
        //     } else {
        //         pwm_g += step_size;
        //     };
        // } else {
        //     channel_b.set_duty(65535 - pwm_b);
        //     if pwm_b as u32 + step_size as u32 > 65535 {
        //         pwm_b = 0;
        //         g_active = true;
        //     } else {
        //         pwm_b += step_size;
        //     };
        // }

        // channel_b.set_duty(pwm_b);
        // if (pwm_b as i32 - step_size as i32) < 1 {
        //     pwm_b = 65000;
        // } else {
        //     pwm_b -= step_size;
        // };
        info!("pwm_r: {}, pwm_g: {}, pwm_b: {}", pwm_r, pwm_g, pwm_b);
    }
}

struct RgbLed<'a> {
    r: u16,
    g: u16,
    b: u16,
    // pwm_slices: Slices,
    pwm_r: &'a Slice<Pwm6, FreeRunning>,
    // channel_r: &'a Channel<Pwm6, FreeRunning, B>,
}

// let pwm_r = &mut pwm_slices.pwm6;
// pwm_r.set_ph_correct();
// pwm_r.enable();
// let channel_r = &mut pwm_r.channel_b;
// channel_r.output_to(led_r);
// channel_r.set_duty(0);

impl RgbLed<'static> {
    pub fn new(mut pwm_slices: Slices) -> RgbLed<'static> {
        let pwm_r: &mut Slice<Pwm6, FreeRunning> = &mut pwm_slices.pwm6.into();
        pwm_r.set_ph_correct();
        pwm_r.enable();
        let mut channel_r = &mut pwm_r.channel_b;

        RgbLed {
            // channel_r,
            pwm_r,
            r: 0,
            g: 0,
            b: 0,
            // pwm_slices,
        }
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // This interrupt routine takes over ownership of the direction pin.
    static mut DIRPIN: Option<DirPinType> = None;
    if DIRPIN.is_none() {
        free(|cs| {
            *DIRPIN = G_DIRPIN.borrow(&cs).take();
        });
    }

    free(|cs| {
        if let Some(ref mut count_pin) = G_COUNTPIN.borrow(cs).borrow_mut().deref_mut() {
            count_pin.clear_interrupt(gpio::Interrupt::EdgeHigh);

            if let Some(d) = DIRPIN {
                if let Some(ref mut count) = G_COUNT.borrow(cs).borrow_mut().deref_mut() {
                    if d.is_high().unwrap() {
                        *count -= 1;
                    } else {
                        *count += 1;
                    }
                }
            }
        }
    });
}

// #[allow(non_snake_case)]
// #[interrupt]
// unsafe fn PIO0_IRQ_0() {
//     static mut BUTTON: Option<CountPinType> = None;
//     static mut COUNTER: Option<i32> = None;
//     if COUNT_PIN.is_none() {
//         free(|cs| {
//             *COUNT_PIN = G_COUNT_PIN.borrow(&cs).take();
//         });
//     }
//     if COUNTER.is_none() {
//         cortex_m::interrupt::free(|cs| {
//             *COUNTER = G_COUNTER.borrow(&cs).take();
//         });
//     }

//     info!("In IRQ");

//     cortex_m::interrupt::free(|cs| {
//         let ref mut g_count = G_COUNTER.borrow(cs).borrow_mut();
//         if let Some(count) = g_count.as_mut() {
//             *count += 1;
//         }
//     });
// }

// cortex_m::interrupt::free(|cs| {});

/*
    let mut encoder_count = pins.gpio8.into_floating_input();
    let mut encoder_direction = pins.gpio9.into_floating_input();
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    encoder_count.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
*/

// let mut led_r = pins.gpio13.into_push_pull_output();
// let mut led_g = pins.gpio14.into_push_pull_output();
// let mut led_b = pins.gpio15.into_push_pull_output();
// let mut e_stop = pins.gpio28.into_floating_input();
// let mut on_off = pins.gpio27.into_floating_input();
// let mut x_pin = pins.gpio10.into_floating_input();
// let mut y_pin = pins.gpio11.into_floating_input();
// let mut z_pin = pins.gpio12.into_floating_input();
// let mut speed_pins = (
//     pins.gpio18.into_floating_input(),
//     pins.gpio19.into_floating_input(),
//     pins.gpio20.into_floating_input(),
//     pins.gpio21.into_floating_input(),
// );
// let mut encoder_direction = pins.gpio9.into_floating_input();

// unsafe {
//     pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
//     pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
// }
// encoder_count_pin.make_interrupt_source(pac::Interrupt::PIO0_IRQ_0);
// encoder_count_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

// loop {
//     let mut is_alarm_finished = false;
//     cortex_m::interrupt::free(|cs| {
//         let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
//         if let Some(alarm0) = g_alarm0.as_mut() {
//             is_alarm_finished = alarm0.finished();
//         }
//     });

//     if is_alarm_finished {
//         cortex_m::interrupt::free(|cs| {
//             let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
//             if let Some(alarm0) = g_alarm0.as_mut() {
//                 let ref mut g_timer = G_TIMER.borrow(cs).borrow_mut();
//                 if let Some(timer) = g_timer.as_mut() {
//                     if G_IS_LED_HIGH.load(Ordering::Acquire) {
//                         alarm0.schedule(500_000.microseconds()).unwrap();
//                         info!("on!");
//                     } else {
//                         alarm0.schedule(1_500_000.microseconds()).unwrap();
//                         info!("off!");
//                     }
//                     alarm0.enable_interrupt(timer);
//                 }
//             }
//         });
//     }

// info!("on!");
// led_pin.set_high().unwrap();
// led_r.set_high().unwrap();
// led_g.set_high().unwrap();
// led_b.set_high().unwrap();
// delay.delay_ms(500);
// info!("off!");
// led_pin.set_low().unwrap();
// delay.delay_ms(500);
//     }
// }

// #[allow(non_snake_case)]
// #[interrupt]
// fn TIMER_IRQ_0() {
//     static mut LED: Option<LEDPinType> = None;
//     if LED.is_none() {
//         cortex_m::interrupt::free(|cs| {
//             *LED = G_LED_PIN.borrow(&cs).take();
//         });
//     }

//     if let Some(led) = LED {
//         let is_high = G_IS_LED_HIGH.load(Ordering::Acquire);
//         if is_high {
//             led.set_low().unwrap();
//         } else {
//             led.set_high().unwrap();
//         }
//         G_IS_LED_HIGH.store(!is_high, Ordering::Release);
//     }

//     cortex_m::interrupt::free(|cs| {
//         let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
//         if let Some(alarm0) = g_alarm0.as_mut() {
//             let ref mut g_timer = G_TIMER.borrow(cs).borrow_mut();
//             if let Some(timer) = g_timer.as_mut() {
//                 alarm0.clear_interrupt(timer);
//             }
//         }
//     });
// }

// // #[allow(non_snake_case)]
// #[interrupt]
// unsafe fn PIO0_IRQ_0() {
//     static mut COUNT_PIN: Option<CountPinType> = None;
//     static mut COUNTER: Option<i32> = None;
//     if COUNT_PIN.is_none() {
//         cortex_m::interrupt::free(|cs| {
//             *COUNT_PIN = G_COUNT_PIN.borrow(&cs).take();
//         });
//     }
//     if COUNTER.is_none() {
//         cortex_m::interrupt::free(|cs| {
//             *COUNTER = G_COUNTER.borrow(&cs).take();
//         });
//     }

//     info!("In IRQ");

//     cortex_m::interrupt::free(|cs| {
//         let ref mut g_count = G_COUNTER.borrow(cs).borrow_mut();
//         if let Some(count) = g_count.as_mut() {
//             *count += 1;
//         }
//     });
// }

// unsafe fn USBCTRL_IRQ() {

//     use core::sync::atomic::{AtomicBool, Ordering};

//     /// Note whether we've already printed the "hello" message.
//     static SAID_HELLO: AtomicBool = AtomicBool::new(false);

//     // Grab the global objects. This is OK as we only access them under interrupt.
//     let usb_dev = USB_DEVICE.as_mut().unwrap();
//     let serial = USB_SERIAL.as_mut().unwrap();

//     // Say hello exactly once on start-up
//     if !SAID_HELLO.load(Ordering::Relaxed) {
//         SAID_HELLO.store(true, Ordering::Relaxed);
//         let _ = serial.write(b"Hello, World!\r\n");
//     }

//     // Poll the USB driver with all of our supported USB Classes
//     if usb_dev.poll(&mut [serial]) {
//         let mut buf = [0u8; 64];
//         match serial.read(&mut buf) {
//             Err(_e) => {
//                 // Do nothing
//             }
//             Ok(0) => {
//                 // Do nothing
//             }
//             Ok(count) => {
//                 // Convert to upper case
//                 buf.iter_mut().take(count).for_each(|b| {
//                     b.make_ascii_uppercase();
//                 });

//                 // Send back to the host
//                 let mut wr_ptr = &buf[..count];
//                 while !wr_ptr.is_empty() {
//                     let _ = serial.write(wr_ptr).map(|len| {
//                         wr_ptr = &wr_ptr[len..];
//                     });
//                 }
//             }
//         }
//     }
// }

// End of file

//
// Author: Declan Vogt
// Purpose:
// Implement a USB pendant that issues key codes that can drive the
// Universal GCode Sender, once that has been configured for the
// same key codes

/*
#include <Encoder.h>    // Used to read CNC pendant-type rotary encoder
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached

const int encoderA = 2;
const int encoderB = 3;

const int axisXPin = 4;
const int axisYPin = 7;
const int axisZPin = 8;

const int Ajog0_01 = 14;
const int Bjog0_1 = 15;
const int Cjog1 = 16;
const int Djog10 = 10;

const int onOff = A2;
const int eStop = A3;
const int feedRatePin = A1;

const int led3r = 5;
const int led3g = 6;
const int led3b = 9;

const char axis = 'X';
const char jogSize = 0;

long count = 0;             // Number of counts that
                            // arive from encoder in 1 tick

// Defines for multi-color status LED
#define RED   0b110000
#define GREEN 0b001100
#define BLUE  0b000011
#define WHITE 0b111111

typedef enum stateType {unset, standby, waiting, running, estop};
stateType state = standby;


// On a rising A edge, if the encoder
// output B is low, then the encoder
// is counting down, else counting up

void encoderAInterrupt(){
    if (digitalRead(encoderB)==HIGH){
        count++;
    } else {
        count--;
    }
}

char readStepsize(){
    bool s1 = !digitalRead(Ajog0_01);
    bool s2 = !digitalRead(Bjog0_1);
    bool s3 = !digitalRead(Cjog1);
    bool s4 = !digitalRead(Djog10);

    int count = 0;
    if (s1) count++;
    if (s2) count++;
    if (s3) count++;
    if (s4) count++;

    if (count!=1){
        return 'E';
    } else if (s1) {
        return '1';
    } else if (s2) {
        return '2';
    } else if (s3) {
        return '3';
    } else {
        return ('4');
    }
}

void driveLEDs(byte rgb){
    if (rgb & 0b110000) {analogWrite(led3r, 127);} else {analogWrite (led3r, 0);}
    if (rgb & 0b001100) digitalWrite(led3g, 1); else digitalWrite (led3g, 0);
    if (rgb & 0b000011) digitalWrite(led3b, 1); else digitalWrite (led3b, 0);
}

bool seen_standby;

void goto_estop(){
    state = eStop;
    seen_standby = false;
}

char readAxis(){
    bool x = !digitalRead(axisXPin);
    bool y = !digitalRead(axisYPin);
    bool z = !digitalRead(axisZPin);

    int count = 0;
    if (x) count++;
    if (y) count++;
    if (z) count++;

    if (count!=1){
        return 'E';
    } else if (x) {
        return'X';
    } else if (y) {
        return 'Y';
    } else {
        return ('Z');
    }
}


void report_state(){
    switch (state) {
    case standby:   Serial.print("S");
                    break;
    case estop:     Serial.print("E");
                    break;
    case waiting:   Serial.print("W");
                    break;
    case running:   Serial.print("R");
                    break;
    }
}

void setup() {
    if (digitalRead(eStop)){
        goto_estop();
    } else {
        state = unset;
    }

    Serial.begin(115200);
    Serial.println("Basic Encoder Test:");

    pinMode(encoderA, INPUT);
    pinMode(encoderB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA), encoderAInterrupt, RISING);

    pinMode(axisXPin, INPUT_PULLUP);
    pinMode(axisYPin, INPUT_PULLUP);
    pinMode(axisZPin, INPUT_PULLUP);

    pinMode(Ajog0_01, INPUT_PULLUP);
    pinMode(Bjog0_1, INPUT_PULLUP);
    pinMode(Cjog1, INPUT_PULLUP);
    pinMode(Djog10, INPUT_PULLUP);

    pinMode(led3r, OUTPUT);
    pinMode(led3g, OUTPUT);
    pinMode(led3b, OUTPUT);

    pinMode(onOff, INPUT_PULLUP);
    pinMode(eStop, INPUT_PULLUP);

//    pinMode(feedRate, INPUT);


}

long oldPos  = 0;
//int led=0;

char currentAxis='X';
char currentStepsize='1';
char currentState = standby;
long countsRemaining = 0;


void change_state(stateType new_state){
    if (new_state != currentState){
        currentState = new_state;
        report_state();
    }
}


int lineCount=0;

void setStopState() {
    Serial.print('S');
    lineCount++;
    state = estop;
    seen_standby = false;
    driveLEDs(RED);
    change_state(state);
}


void loop() {
    delay(20);
    if (digitalRead(eStop)){
        setStopState();
    }
    if (state!=estop){
        if (digitalRead(onOff)){
            change_state(standby);
            driveLEDs(BLUE);
        } else {
            change_state(waiting);
            driveLEDs(WHITE);
        }
    } else {
        if (digitalRead(onOff)){
            //state = standby;
            seen_standby = true;
        } else {
            if (seen_standby) {
                seen_standby = false;
                change_state(waiting);
            }
        }
    }

    report_state();

    char axis = readAxis();
    char stepsize = readStepsize();
 /*   if (axis == 'E') setStopState();
    if (stepsize == 'E') setStopState();
*/

    if (count!=0){
        countsRemaining+= count;
    }

    if (countsRemaining<0){
        Serial.print('-');
        countsRemaining += 1;
        lineCount++;
    } else if (countsRemaining>0){
        Serial.print('+');
        countsRemaining -= 1;
        lineCount++;
    }


    //Serial.print('_');
    if ((axis!='E')&&(currentAxis!=axis)){
        Serial.print(axis);
        lineCount++;
        currentAxis = axis;
    }
    if (stepsize!='E')&&(currentStepsize!=stepsize)){
        Serial.print(stepsize);
        currentStepsize = stepsize;
        lineCount++;
    }

  //  Serial.print(' ');
  //  Serial.print(analogRead(feedRatePin));
    if (lineCount>79){
        lineCount=0;
        Serial.println();
    }
    count=0;
}

*/
