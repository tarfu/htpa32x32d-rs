
use crate::Error;
use hal;
use crate::interface::SensorInterface;

pub struct I2cInterface<I2C> {
    i2c: I2C,
    addr_sensor: u8,
    #[allow(dead_code)]
    addr_eeprom: u8,
    woken_up: bool
}

impl<I2C, CommE> I2cInterface<I2C>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>,
{
    /// Create new SSD1306 I2C interface
    pub fn new(i2c: I2C, addr_sensor: u8, addr_eeprom: u8) -> Self {
        Self { i2c: i2c, addr_sensor: addr_sensor, addr_eeprom: addr_eeprom, woken_up: false}
    }
}

impl<I2C, CommE> SensorInterface for I2cInterface<I2C>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>,
{
    type Error = Error<CommE, ()>;

    fn is_woken_up(&self) -> bool {
        self.woken_up
    }

    fn set_wake_up(&mut self, wakeup: bool) {
        self.woken_up = wakeup;
    }

    fn send_commands(&mut self, cmds: &[u8]) -> Result<(), Self::Error> {
        // Copy over given commands to new aray to prefix with command identifier
        let mut writebuf: [u8; 2] = [0; 2];
        writebuf[1..=cmds.len()].copy_from_slice(&cmds[0..cmds.len()]);

        self.i2c
            .write(self.addr_sensor, &writebuf[..=cmds.len()])
            .map_err(Error::Comm)
    }



}



