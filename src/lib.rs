//! Driver for the HTPA32x32 an Thermopile Array from Heimann

#![no_std]
// #![deny(missing_debug_implementations)]
//#![deny(missing_docs)] // just allow for missing docs right now
#![allow(missing_docs)]
#![deny(warnings)]
#![deny(missing_copy_implementations)]
#![deny(trivial_casts)]
#![deny(trivial_numeric_casts)]
#![deny(unsafe_code)]
#![deny(unstable_features)]
#![deny(unused_import_braces)]
#![deny(unused_qualifications)]


use embedded_hal as hal;

//use libm;
//use bit_field::BitField;

use crate::hal::blocking::delay::DelayMs;
use crate::hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::blocking::i2c::Read;


pub struct Htpa32x32d<I2C> {

    i2c: I2C,
    addr_sensor: u8,
    #[allow(dead_code)]
    addr_eeprom: u8,
    woken_up: bool,
}


impl<I2C, E> Htpa32x32d<I2C>
    where
        I2C: Write<Error = E> + WriteRead<Error = E> + Read<Error = E>,
{
    /// Create new htpa32x32d I2C interface
    /// A maximum of 1mhz i2c is supported, typical 400khz
    pub fn new(i2c: I2C, addr_sensor: u8, addr_eeprom: u8) -> Self {

        let sensor = Self{ i2c, addr_sensor, addr_eeprom, woken_up: false };

        return sensor
    }


    pub fn is_woken_up(&self) -> bool {
        self.woken_up
    }

    pub fn suspend(&mut self) {
        self.woken_up = false;
        self.i2c.write(self.addr_sensor, &[Register::Configuration as u8, self.woken_up as u8]).err();
    }

    pub fn wakeup_and_write_config<DELAY>(&mut self, delay: &mut DELAY) -> Result<(), Error<E>>
        where DELAY: DelayMs<u8> {

        let clk = ClkTrim::from(0x14);

        self.woken_up=true;
        self.i2c.write(self.addr_sensor, &[Register::Configuration as u8, self.woken_up as u8]).err();
        delay.delay_ms(30);
        self.i2c.write(self.addr_sensor, &[Register::Trim1 as u8, 0x0C]).err();
        delay.delay_ms(5);
        self.i2c.write(self.addr_sensor, &[Register::Trim2 as u8, 0x0C]).err();
        delay.delay_ms(5);
        self.i2c.write(self.addr_sensor, &[Register::Trim3 as u8, 0x0C]).err();
        delay.delay_ms(5);
        self.i2c.write(self.addr_sensor, &[Register::Trim4 as u8, clk.0]).err();
        delay.delay_ms(5);
        self.i2c.write(self.addr_sensor, &[Register::Trim5 as u8, 0x0C]).err();
        delay.delay_ms(5);
        self.i2c.write(self.addr_sensor, &[Register::Trim6 as u8, 0x0C]).err();
        delay.delay_ms(5);
        self.i2c.write(self.addr_sensor, &[Register::Trim7 as u8, 0x88]).map_err(Error::I2c)

    }

    pub fn start_measurement(&mut self, measurement: Measurement) -> Result<(), Error<E>> {
        if !self.woken_up { return Err(Error::Standby) }
        let configuration_mask = measurement.get_configuration_bitmask();

        self.i2c
            .write(self.addr_sensor, &[Register::Configuration as u8, configuration_mask])
            .map_err(Error::I2c)
    }

    pub fn stop_measurement(&mut self)  -> Result<(), Error<E>> {
        if !self.woken_up { return Err(Error::Standby) }

        self.i2c.write(self.addr_sensor, &[Register::Configuration as u8, self.woken_up as u8])
            .map_err(Error::I2c)

    }

    pub fn check_measurement_ready(&mut self, measurement: Measurement) -> Result<bool, Error<E>> {
        let status = match self.get_sensor_status() {
            Ok(t) => t,
            Err(e) => return Err(e)
        };
        Ok(measurement.check_measurement_readiness(status))

    }


    fn get_sensor_status(&mut self) -> Result<u8, Error<E>> {
        let writebuf: [u8; 1] = [Register::Status as u8];
        let mut readbuf: [u8; 1] = [0; 1];


        match self.i2c.write_read(self.addr_sensor, &writebuf, &mut readbuf) {
            Ok(_) => Ok(readbuf[0]),
            Err(e) => Err(Error::I2c(e))
        }
    }

    /// get measurement for the block specified in the former send command and select from which half it should come
    /// data must be at 258 in size
    pub fn get_measurement_data(&mut self, half: SensorHalf, data: &mut [u8; 258]) -> Result<(), Error<E>> {
        let sensor_half = [half as u8];

        self.i2c.write_read(self.addr_sensor, &sensor_half, data)
            .map_err(Error::I2c)
    }


}

#[derive(Debug, Clone, Copy)]
pub enum Measurement {
    /// After the START bit is set the chip starts a conversion of the array or blind elements and  enters the idle state (not sleep!) when finished. The BLOCK selects one of the four multiplexed array blocks.
    Ptat {block: Block },
    /// sample electrical offset instead of active pixels ignoring block
    Blind {},
    ///  VDD voltage is measured instead of the PTAT value
    VddMeas {block: Block },
}

impl Measurement {
    fn get_configuration_bitmask(self) -> u8 {
        match self {
            Measurement::Ptat {block} => (block as u8) << 4 | 1 << 3 | 1,
            Measurement::VddMeas {block} => (block as u8) << 4 | 1 << 3 | 1 << 2 | 1,
            Measurement::Blind {} => 1 << 3 | 1 << 1 | 1
        }
    }

    fn check_measurement_readiness(self, status: u8) -> bool {
        match self {
            Measurement::Ptat {block } => (
                ((!(StatusBitmasks::Rfu as u8))  & status)
                    ==
                    ((block as u8) << 4) | (StatusBitmasks::Eoc as u8)) ,
            Measurement::Blind {} => (
                (((!(StatusBitmasks::Rfu as u8)) | (!(StatusBitmasks::Block as u8)))  & status)
                    ==
                    (StatusBitmasks::Blind as u8 | (StatusBitmasks::Eoc as u8))),
            Measurement::VddMeas {block } => (
                ((!(StatusBitmasks::Rfu as u8)) & status)
                    ==
                    (((block as u8) << 4) | (StatusBitmasks::VddMeas as u8) | (StatusBitmasks::Eoc as u8))),
        }
    }

}

/// Addresses for the Sensor and EEPROM
#[derive(Debug, Clone, Copy)]
pub enum Address{
    /// Address for the sensor
    Sensor = 0x1A,
    /// Address for the eeprom to store calibration data
    Eeprom = 0x50
}

/// Commands available for the Sensor


enum StatusBitmasks {
    /// EOC flag is set a previous started conversion has been finished
    Eoc = 0b0000_0001,
    /// Indicates that the operation was Blind
    Blind = 0b0000_0010,
    /// Indicates that the operation was VDD_MEAS
    VddMeas = 0b0000_0100,
    /// Block for which the operation was done
    Block = 0b0011_0000,
    /// Reserved for future use bits
    Rfu = 0b1100_1000,
}

#[allow(dead_code)]
enum Register {
    /// Configuration for commands
    Configuration = 0x01,
    /// Readyness and sensor status
    Status = 0x02,
    /// Amplification and ADC resolution
    Trim1 = 0x03,
    /// BIAS_TRIM_TOP  bias current of the ADC. A faster clock frequency requires higher bias current setting.
    Trim2 = 0x04,
    /// BIAS_TRIM_BOT
    Trim3 = 0x05,
    /// CLK_TRIM: 0 to 63 -> 1MHz to 13MHz
    Trim4 = 0x06,
    /// BPA_TRIM_TOP  adjust the common mode voltage of the preamplifier
    Trim5 = 0x07,
    /// BPA_TRIM_BOT
    Trim6 = 0x08,
    /// PU_SDA_TRIM, PU_SCL_TRIM -- 1000 = 100 kOhm; 0100 = 50 kOhm; 0010 = 10 kOhm; 0001 = 1 kOhm
    Trim7 = 0x09,
    /// Read block from top half of the sensor
    ReadTop = 0x0A,
    ///  Read block from bottom half of the sensor
    ReadBottom = 0x0B,
}

#[derive(Debug, Clone, Copy)]
pub struct ClkTrim(u8);

impl From<u8> for ClkTrim {
    fn from(val: u8) -> ClkTrim {
        if val > 63 {
            return ClkTrim(63);
        }
        return ClkTrim(val);
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Block {
    Block0 = 0b00,
    Block1 = 0b01,
    Block2 = 0b10,
    Block3 = 0b11,
}

impl From<u8> for Block {
    fn from(val: u8) -> Block {
        match val {
            0 => Block::Block0,
            1 => Block::Block1,
            2 => Block::Block2,
            3 => Block::Block3,
            _ => panic!("Block too high"),
        }
    }
}

/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// Sensor is in Standby
    Standby,
}


#[derive(Clone, Copy)]
pub enum SensorHalf{
    Top = 0x0A,
    Bottom =0x0B,
}


#[cfg(test)]
mod tests {
// TODO :D
//    #[test]
//    fn it_works() {
//        assert_eq!(2 + 2, 4);
//    }
}
