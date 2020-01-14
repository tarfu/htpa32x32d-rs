#![no_std]
#![no_main]

extern crate embedded_graphics;
extern crate panic_semihosting;

use core::fmt::{Error, Write};
use core::ptr::write;

use cortex_m_rt::entry;

use cortex_m;

use embedded_graphics::fonts::Font6x8;
use embedded_graphics::image::Image16BPP;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Drawing;

use nrf52840_hal::{
    clocks, delay,
    gpio::{
        self,
        p0::{self, *},
        *,
    },
    nrf52840_pac as pac,
    spim::{self, Spim},
    target,
    twim::{self, Twim},
};

use htpa32x32d::Block::*;
use htpa32x32d::{Address, Htpa32x32d, Measurement, SensorHalf};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::OutputPin;
use st7735_lcd;
use st7735_lcd::Orientation;

use ssd1306::prelude::*;
use ssd1306::Builder;

///
/// D22 .. D23 (aka I2C pins)
/// D22 is P0.12 (SDA)
/// D23 is P0.11 (SCL)
///
// D24 .. D26 (aka SPI pins)
// D24 is P0.15 (SPI MISO)
/// D25 is P0.13 (SPI MOSI)
/// D26 is P0.14 (SPI SCK )
/// D11 is P0.06
/// D12 is P0.08
/// D13 is P1.09
#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let port0 = p.P0.split();
    let port1 = p.P1.split();

    let cp = cortex_m::Peripherals::take().unwrap();

    let mut led1: Pin<Output<PushPull>> = port1.p1_10.into_push_pull_output(Level::High).degrade();
    let mut led2: Pin<Output<PushPull>> = port1.p1_15.into_push_pull_output(Level::High).degrade();

    let spiclk = port0.p0_14.into_push_pull_output(Level::Low).degrade();
    let spimosi = port0.p0_13.into_push_pull_output(Level::Low).degrade();
    let spimiso = port0.p0_15.into_floating_input().degrade();

    let mut select = port0.p0_08.into_push_pull_output(Level::Low).degrade();
    let mut rst = port0.p0_06.into_push_pull_output(Level::Low).degrade();
    let mut chip_select = port1.p1_09.into_push_pull_output(Level::Low).degrade();

    let pins = spim::Pins {
        sck: spiclk,
        miso: Some(spimiso),
        mosi: Some(spimosi),
    };

    let mut spi = Spim::new(p.SPIM2, pins, spim::Frequency::M32, spim::MODE_2, 0);

    let mut delay = delay::Delay::new(cp.SYST);

    let mut disp_lcd = st7735_lcd::ST7735::new(spi, select, rst, true, false);
    disp_lcd.init(&mut delay).unwrap();
    disp_lcd.set_orientation(&Orientation::Landscape).unwrap();

    let scl = port0.p0_11.into_floating_input().degrade();
    let sda = port0.p0_12.into_floating_input().degrade();

    let pinsTwim = twim::Pins { scl, sda };

    let i2c = Twim::new(p.TWIM0, pinsTwim, twim::Frequency::K400);

    let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);
    let mut temp_sensor: Htpa32x32d<_> = Htpa32x32d::new(
        manager.acquire(),
        Address::Sensor as u8,
        Address::Eeprom as u8,
    );

    temp_sensor.wakeup_and_write_config(&mut delay);

    //    let mut disp: GraphicsMode<_> = Builder::new().with_size(DisplaySize::Display128x32).with_rotation(DisplayRotation::Rotate180).connect_i2c(manager.acquire()).into();
    //
    //    disp.init().expect("Display initialization");
    //    disp.flush().expect("Cleans the display");
    //
    let wokenString = Font6x8::render_str("woken")
        .stroke(Some(Rgb565::from((0, 0, 255))))
        .fill(Some(Rgb565::from((255, 255, 255))))
        .into_iter();
    let startedString = Font6x8::render_str("started")
        .stroke(Some(Rgb565::from((0, 0, 255))))
        .fill(Some(Rgb565::from((255, 255, 255))))
        .into_iter();

    let readyString = Font6x8::render_str("ready")
        .stroke(Some(Rgb565::from((0, 0, 255))))
        .fill(Some(Rgb565::from((255, 255, 255))))
        .into_iter();
    //    disp.set_pixel(8, 8, 1);
    //
    //    disp.flush().expect("Render display");

    let black_backdrop: Rectangle<Rgb565> =
        Rectangle::new(Coord::new(0, 0), Coord::new(160, 40)).fill(Some(Rgb565(0x0000)));

    let white_backdrop: Rectangle<Rgb565> = Rectangle::new(Coord::new(0, 0), Coord::new(160, 40))
        .fill(Some(Rgb565::from((255, 255, 255))));

    disp_lcd.draw(black_backdrop.into_iter());
    disp_lcd.draw(white_backdrop.into_iter());

    let mut messure: Measurement = Measurement::Ptat { block: Block0 };
    let mut data: [u8; 258] = [255u8; 258];
    let mut waiter = 0u16;
    let mut run = 0;

    loop {

        let mut buffer: heapless::String<heapless::consts::U64> = heapless::String::new();
        write!(buffer, "{}", run,).unwrap();
        disp_lcd.draw(
            Font6x8::render_str(&buffer)
                .stroke(Some(Rgb565::from((0, 0, 255))))
                .fill(Some(Rgb565::from((255, 255, 255))))
                .translate(Coord::new(64, 0))
                .into_iter(),
        );

        led1.set_low();
        delay.delay_ms(128 as u8);
        led1.set_high();
        delay.delay_ms(128 as u8);
        if temp_sensor.is_woken_up() && run == 0 {
            disp_lcd.draw(wokenString);
        }

        messure = Measurement::Ptat { block: Block0 };
        temp_sensor.start_measurement(messure);
        if run == 0 {
            disp_lcd.draw(startedString);
        }
        delay.delay_ms(30 as u8);
        while !temp_sensor.check_measurement_ready(messure).unwrap() {
            //            disp_lcd.set_pixel(waiter%160, (waiter/160)%128, 0xffff);
            //            waiter += 1;
            led1.set_low();
            delay.delay_ms(128 as u8);
            led1.set_high();
            delay.delay_ms(128 as u8);
        }

        //        for i in 0..waiter {
        //            disp_lcd.set_pixel((waiter-i)%160, ((waiter-i)/160)%128, 0x0000);
        //        }
        waiter = 0;

        temp_sensor
            .get_measurement_data(SensorHalf::Top, &mut data)
            .unwrap();
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }
        temp_sensor.get_measurement_data(SensorHalf::Bottom, &mut data);
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                31 - num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }

        delay.delay_ms(128 as u8);

        messure = Measurement::Ptat { block: Block1 };
        temp_sensor.start_measurement(messure);
        delay.delay_ms(30 as u8);
        while !temp_sensor.check_measurement_ready(messure).unwrap() {
            led1.set_low();
            delay.delay_ms(128 as u8);
            led1.set_high();
            delay.delay_ms(128 as u8);
        }
        temp_sensor
            .get_measurement_data(SensorHalf::Top, &mut data)
            .unwrap();
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                4 + num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }
        temp_sensor.get_measurement_data(SensorHalf::Bottom, &mut data);
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                31 - 4 - num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }

        delay.delay_ms(128 as u8);

        messure = Measurement::Ptat { block: Block2 };
        temp_sensor.start_measurement(messure);
        delay.delay_ms(30 as u8);
        while !temp_sensor.check_measurement_ready(messure).unwrap() {
            led1.set_low();
            delay.delay_ms(128 as u8);
            led1.set_high();
            delay.delay_ms(128 as u8);
        }
        temp_sensor
            .get_measurement_data(SensorHalf::Top, &mut data)
            .unwrap();
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                8 + num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }
        temp_sensor.get_measurement_data(SensorHalf::Bottom, &mut data);
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                31 - 8 - num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }

        delay.delay_ms(128 as u8);

        messure = Measurement::Ptat { block: Block3 };
        temp_sensor.start_measurement(messure);
        delay.delay_ms(30 as u8);
        while !temp_sensor.check_measurement_ready(messure).unwrap() {
            led1.set_low();
            delay.delay_ms(128 as u8);
            led1.set_high();
            delay.delay_ms(128 as u8);
        }
        temp_sensor
            .get_measurement_data(SensorHalf::Top, &mut data)
            .unwrap();
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                12 + num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }
        temp_sensor.get_measurement_data(SensorHalf::Bottom, &mut data);
        for num1 in 0..128 {
            disp_lcd.set_pixel(
                num1 % 32,
                31 - 12 - num1 / 32,
                (data[(2 + num1 * 2) as usize] as u16) << 8
                    | (data[(3 + num1 * 2) as usize] as u16),
            );
        }

        delay.delay_ms(128 as u8);

        run += 1;
    }
}
