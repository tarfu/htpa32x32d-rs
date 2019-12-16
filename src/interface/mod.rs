pub mod i2c;


pub trait SensorInterface{
    type Error;

    fn is_woken_up(&self) -> bool;
    fn set_wake_up(&mut self, wakeup: bool);
    fn send_commands(&mut self, cmd: &[u8]) -> Result<(), Self::Error>;
}