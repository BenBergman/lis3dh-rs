//#![deny(missing_docs)]
//#![deny(warnings)]
#![no_std]

extern crate embedded_hal;

use embedded_hal::blocking::i2c::{WriteRead};
use self::Register::*;

#[derive(Copy, Clone)]
enum Register {
    WHOAMI = 0x0F,
    //LIS3DH_REG_CTRL1 = 0x20,
}

impl Register {
    fn addr(&self) -> u8 {
        *self as u8
    }
}

#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data.
    WrongAddress,
}

pub struct Lis3dh<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Lis3dh<I2C>
where
    I2C: WriteRead<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> Result<Self, Error<E>> {
        let mut lis3dh = Lis3dh { i2c, address };

        let buf = lis3dh.read_register(WHOAMI)?;

        if buf == 0x33 {
            Ok(lis3dh)
        } else {
            Err(Error::WrongAddress)
        }
    }

    fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }
}

/************************/

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
