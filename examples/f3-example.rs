//! This example reads the sensor Htpa32x32d and displays them on a st7735 display
//!
//! This example is runs on the STM32F3 Discovery board using I2C1.
//!
//! ```
//! F3    <-> Htpa32x32d <-> ssd1306
//! GND   <-> GND        <-> GND
//! +3.3V <-> +3.3V      <-> +3.3V
//! PB7   <-> SDA        <-> SDA
//! PB6   <-> SCL        <-> SCL
//! ```
//! Run with:
//! `cargo run --example f3-example --target thumbv7em-none-eabihf`

#![deny(unsafe_code)]
#![no_std]
#![no_main]

// panic handler
extern crate embedded_graphics;
extern crate panic_semihosting;


use cortex_m_rt::entry;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;
use f3::{
    hal::{delay::Delay, i2c::I2c, prelude::*, stm32f30x},
    hal::time,
    led::Led,
};
use ssd1306::prelude::*;
use ssd1306::Builder;

use cortex_m_semihosting::hprintln;
use cortex_m_semihosting::hio;



use htpa32x32d::interface::i2c::I2cInterface;


use core::fmt::{Write, Error};
use embedded_graphics::pixelcolor::{PixelColorU8, PixelColorU16};
use f3::hal::stm32f30x::SPI1;
use f3::hal::spi::Spi;
use embedded_hal::spi::{Mode, Polarity, Phase};
use st7735_lcd::Orientation;
use st7735_lcd;
use embedded_hal::digital::v2::OutputPin;
use f3::hal::gpio::{Output, PushPull};
use embedded_graphics::image::Image16BPP;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rect;
use core::ptr::write;
use htpa32x32d::{Htpa32x32d, Address};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let clocks = rcc.cfgr.sysclk(72.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    let mut led: Led = gpioe
        .pe9
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
        .into();
    let mut delay = Delay::new(cp.SYST, clocks);


    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

//    let sck = gpiob.pb4.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
//    let sda = gpiob.pb5.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);


    let mut rst = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut select = gpioa
        .pa3
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut chip_select = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    // SPI
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        Mode{phase: Phase::CaptureOnFirstTransition, polarity: Polarity::IdleHigh},
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut disp_lcd = st7735_lcd::ST7735::new(spi, select, rst, false, false);
    disp_lcd.init(&mut delay).unwrap();
    disp_lcd.set_orientation(&Orientation::Landscape).unwrap();

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

    let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);
    let mut temp_sensor: Htpa32x32d<_, _> = Htpa32x32d::new(manager.acquire(), delay, Address::Sensor as u8, Address::Eeprom as u8);
    let mut disp: GraphicsMode<_> = Builder::new().with_size(DisplaySize::Display128x32).with_rotation(DisplayRotation::Rotate180).connect_i2c(manager.acquire()).into();

    disp.init().unwrap();
    disp.flush().unwrap();


    let mut stdout = match hio::hstdout() {
        Ok(fd) => fd,
        Err(()) => return Err(core::fmt::Error),
    };


    let clock = time::MonoTimer::new(cp.DWT, clocks);
    let mut last_instant = clock.now();
    let clock_divider: time::Hertz = 72.khz().into();
    let mut led_on = false;
    loop {

        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led_on = match led_on {
            true => {led.off(); false},
            false => {led.on(); true},
        };



        let mut buffer: heapless::String<heapless::consts::U64> = heapless::String::new();
        let mut data = [0; READ_SAMPLES];


//            disp.clear();

            let now_instant = clock.now();

            write!(buffer, "{} {} {} {}         ", last_instant.elapsed()/ clock_divider.0, 1, 1, 1).unwrap();
            disp.draw(
                Font6x8::render_str(&buffer)
                    .with_stroke(Some(1u8.into()))
                    .into_iter(),
            );
            last_instant = now_instant;

            disp.set_pixel(xPointer, now, 1);
            lastPoint = now;

            disp.flush().unwrap();


        }
    }
}
