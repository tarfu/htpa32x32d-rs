


#[test]
fn it_works() {
    assert_eq!(2 + 2, 4);
}

#[test]
fn i2c_example() {
    extern crate embedded_hal;
    extern crate embedded_hal_mock;

    //    use embedded_hal::prelude::*;
    use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

// Configure expectations
    let expectations = [
        I2cTransaction::write(0xaa, vec![1, 2]),
        I2cTransaction::read(0xbb, vec![3, 4]),
        I2cTransaction::write_read(0xaa, vec![1, 2], vec![3, 4])
    ];
    let mut i2c = I2cMock::new(&expectations);

// Writing
    i2c.write(0xaa, &[1, 2]).unwrap();

// Reading
    let mut buf = vec![0u8; 2];
    i2c.read(0xbb, &mut buf).unwrap();
    assert_eq!(buf, vec![3, 4]);

    let mut buf = vec![0u8; 2];
    i2c.write_read(0xaa, &[1, 2], &mut buf).unwrap();
    assert_eq!(buf, vec![3, 4]);


// Finalise expectations
    i2c.done();
}