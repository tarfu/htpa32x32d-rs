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

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),
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

#[derive(Clone, Copy)]
pub enum Command{
    /// Start the sensor for operation
    Wakeup(bool),
    /// After the START bit is set the chip starts a conversion of the array or blind elements and  enters the idle state (not sleep!) when finished. The BLOCK selects one of the four multiplexed array blocks.
    Ptat{start: bool, block: Block},
    /// sample electrical offset instead of active pixels ignoring block
    Blind{start: bool},
    ///  VDD voltage is measured instead of the PTAT value
    VddMeas{start: bool, block: Block},
}
impl Command {
    pub fn send<SI>(self, iface: &mut SI) -> Result<(), SI::Error>
        where
            SI: SensorInterface,
    {

        // Transform command into a fixed size array of 7 u8 and the real length for sending
        let (data, len) = match self {
            Command::Wakeup(wakeup) => {iface.set_wake_up(wakeup); ([0x01, wakeup as u8], 2)},
            Command::Ptat{start, block} => ([0x01, (block as u8) << 4 | (start as u8) << 3 | (iface.is_woken_up() as u8)], 2),
            Command::Blind {start} => ([0x01, (start as u8) << 3 | 1 << 1 | (iface.is_woken_up() as u8)], 2),
            Command::VddMeas {start, block} => ([0x01, (block as u8) << 4 | (start as u8) << 3 | 1 << 2 | (iface.is_woken_up() as u8)], 2),
        };

        iface.send_commands(&data[0..len])

    }


}
#[derive(Debug, Clone, Copy)]
pub struct ClkTrim(u8);
impl From<u8> for ClkTrim{
    fn from(val: u8) -> ClkTrim{
        if val > 63 {
            panic!("Invalid Clock Trim!")
        }
        return ClkTrim(val)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum  Block {
    Block0 = 0b00,
    Block1 = 0b01,
    Block2 = 0b10,
    Block3 = 0b11
}

impl From<u8> for Block{
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



extern crate embedded_hal as hal;

use crate::interface::SensorInterface;


pub mod interface;


#[cfg(test)]
mod tests {
// TODO :D
//    #[test]
//    fn it_works() {
//        assert_eq!(2 + 2, 4);
//    }
}
