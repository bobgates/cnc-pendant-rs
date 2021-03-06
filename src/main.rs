//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.

#![no_std]
#![no_main]

mod inputs;
mod rgb_led;

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::delay::Delay;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
// use embedded_error;
use embedded_hal::adc::OneShot;
use embedded_hal::{digital::v2::InputPin, prelude::_embedded_hal_timer_CountDown}; //, OutputPin};
use embedded_time::{duration::Extensions, fixed_point::FixedPoint};
use nb;
use panic_halt as _;

use rgb_led::RgbLed;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use rp_pico::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        self, bank0::Gpio25, bank0::Gpio8, bank0::Gpio9, Floating, Input, Output, Pin, PushPull,
    },
    pac::{self, interrupt},
    sio::Sio,
    watchdog::Watchdog,
    Adc,
};

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

type LEDPinType = Pin<Gpio25, Output<PushPull>>;
static G_LED_PIN: Mutex<RefCell<Option<LEDPinType>>> = Mutex::new(RefCell::new(None));

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

    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let _delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        USB_BUS = Some(usb_bus);
    }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let serial = SerialPort::new(&bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }
    // let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

    let usb_dev = UsbDeviceBuilder::new(&bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Road Running Inc")
        .product("Serial port")
        .serial_number("Beep-beep")
        .device_class(2)
        .build();
    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = pins.gpio26.into_floating_input();

    let led_r = pins.gpio13;
    let led_g = pins.gpio14;
    let led_b = pins.gpio15;

    // Setup PWMs
    // There are 8 PWM slices, each with A and B channels
    // r, gpio13, is on slice 6, channel A,
    // g, gpio14, is on slice 7, channel A
    // b, gpio15, is on slice 7, channel B

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut rgb_leds = RgbLed::new(pwm_slices, led_r, led_g, led_b);

    // Pins and definitions for rotary encoder
    let count_pin: CountPinType = pins.gpio8.into_floating_input();
    let count = 0;
    let dir_pin: DirPinType = pins.gpio9.into_floating_input();

    let mut xyz_input = inputs::Xyz::new(
        pins.gpio10.into_pull_up_input(),
        pins.gpio11.into_pull_up_input(),
        pins.gpio12.into_pull_up_input(),
    );

    let mut speed_input = inputs::Stepsize::new(
        pins.gpio18.into_pull_up_input(),
        pins.gpio19.into_pull_up_input(),
        pins.gpio20.into_pull_up_input(),
        pins.gpio21.into_pull_up_input(),
    );

    let e_stop_pin = pins.gpio28.into_pull_up_input();
    let on_off_pin = pins.gpio27.into_pull_up_input();

    // let e_stop = false;
    // let mut on = false;
    let mut old_e_stop = false;
    let mut old_on = false;
    let mut old_pot: u16 = 0;

    let mut old_speed: Option<inputs::Speed> = None;
    let mut old_axis: Option<inputs::Axis> = None;

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    }

    free(|cs| {
        G_DIRPIN.borrow(cs).replace(Some(dir_pin));
        G_COUNTPIN.borrow(cs).replace(Some(count_pin));
        G_COUNT.borrow(cs).replace(Some(count));
    });
    let mut old_count: i32 = 0;

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    count_down.start(2000.milliseconds());
    let _ = nb::block!(count_down.wait());
    // let _ = serial.write(b"Pendant alive\r\n");

    let mut _vcount: u32 = 0;

    loop {
        count_down.start(20.milliseconds());
        free(|cs| {
            if let Some(count) = G_COUNT.borrow(cs).borrow_mut().deref_mut() {
                if old_count != *count {
                    old_count = *count;
                    info!("Count: {}", old_count);
                }
            };
        });
        // delay.delay_ms(20);

        rgb_leds.color_demo();

        let e_stop = e_stop_pin.is_high().unwrap();
        if old_e_stop != e_stop {
            info!("E stop: {}", e_stop);
            old_e_stop = e_stop;
        }

        let on = on_off_pin.is_low().unwrap();
        if old_on != on {
            info!("On: {}", on);
            old_on = on;
        }

        let pot: u16 = adc.read(&mut adc_pin_0).unwrap();
        let diff = if old_pot >= pot {
            old_pot - pot
        } else {
            pot - old_pot
        };
        if diff > 100 {
            info!("Pot: {}", pot);
            old_pot = pot;
        }

        let a = speed_input.scan();
        if a != old_speed {
            info!("{}", inputs::Speed::report(a));
            old_speed = a;
        }
        let b = xyz_input.scan();
        if b != old_axis {
            info!("{}", inputs::Axis::report(b));
            old_axis = b;
        }

        let _ = nb::block!(count_down.wait());
        let _ = count_down.wait();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // This interrupt routine takes over ownership of the direction pin.
    static mut DIRPIN: Option<DirPinType> = None;
    if DIRPIN.is_none() {
        free(|cs| {
            *DIRPIN = G_DIRPIN.borrow(cs).take();
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

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_0() {
    static mut LED: Option<LEDPinType> = None;

    if LED.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED = G_LED_PIN.borrow(cs).take();
        });
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Note whether we've already printed the "hello" message.
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();

    let serial = USB_SERIAL.as_mut().unwrap();

    // Say hello exactly once on start-up
    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = serial.write(b"Hello, World!\r\n");
    }

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                // Convert to upper case
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });

                // Send back to the host
                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                    });
                }
            }
        }
    }
}

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
