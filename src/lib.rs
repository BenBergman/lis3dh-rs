#![no_std]
#![allow(non_camel_case_types)]

mod register;

use core::fmt::Debug;
use core::convert::TryInto;
use embedded_hal::blocking::i2c::{WriteRead, Write};

pub use register::Register;
pub use accelerometer;
use accelerometer::{I16x3, Accelerometer, Tracker};

#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data.
    WrongAddress,
    WriteToReadOnly,
    InvalidDataRate,
    InvalidRange,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Range {
    G16 = 0b11,  //  +/- 16g
    G8  = 0b10,  //  +/-  8g
    G4  = 0b01,  //  +/-  4g
    G2  = 0b00,  //  +/-  2g
    Invalid = 0xff,
}

impl Range {
    pub fn bits(self) -> u8 {
        self as u8
    }

    fn from(value: u8) -> Range {
        match value {
            0b11 => Range::G16,
            0b10 => Range::G8,
            0b01 => Range::G4,
            0b00 => Range::G2,
            _ => Range::Invalid
        }
    }
}


#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DataRate {
    Hz_1344_LP5k   = 0b1001, // 1344Hz in normal mode, 5KHz in low power mode
    Hz_400         = 0b0111,
    Hz_200         = 0b0110,
    Hz_100         = 0b0101,
    Hz_50          = 0b0100,
    Hz_25          = 0b0011,
    Hz_10          = 0b0010,
    Hz_1           = 0b0001,
    PowerDown      = 0b0000,
    LowPower_1K6HZ = 0b1000,
    Invalid        = 0xff,
}

impl DataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }

    fn from(value: u8) -> DataRate {
        match value {
            0b1001 => DataRate::Hz_1344_LP5k,
            0b0111 => DataRate::Hz_400,
            0b0110 => DataRate::Hz_200,
            0b0101 => DataRate::Hz_100,
            0b0100 => DataRate::Hz_50,
            0b0011 => DataRate::Hz_25,
            0b0010 => DataRate::Hz_10,
            0b0001 => DataRate::Hz_1,
            0b0000 => DataRate::PowerDown,
            0b1000 => DataRate::LowPower_1K6HZ,
            _ => DataRate::Invalid,
        }
    }
}

pub struct Lis3dh<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>
{
    pub fn new(i2c: I2C, address: u8) -> Result<Self, Error<E>> {
        let mut lis3dh = Lis3dh { i2c, address };

        let buf = lis3dh.read_register(Register::WHOAMI)?;

        if buf != 0x33 {
            return Err(Error::WrongAddress)
        }
        // Enable all axes, normal mode.
        lis3dh.write_register(Register::CTRL1, 0x07)?;
        // Set 400Hz data rate.
        lis3dh.set_datarate(DataRate::Hz_400)?;
        // High res & BDU enabled
        lis3dh.write_register(Register::CTRL4, 0x88)?;
        // Enable ADCs.
        lis3dh.write_register(Register::TEMP_CFG, 0x80)?;
        // Latch interrupt for INT1
        lis3dh.write_register(Register::CTRL5, 0x08)?;
        Ok(lis3dh)
    }

    pub fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<E>> {
        if datarate == DataRate::Invalid {
            return Err(Error::InvalidDataRate);
        }
        let mut ctrl1 = self.read_register(Register::CTRL1)?;
        ctrl1 &= !0xf0;
        ctrl1 |= datarate.bits() << 4;
        self.write_register(Register::CTRL1, ctrl1)
    }

    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        Ok(DataRate::from((ctrl1 >> 4) & 0x0F))
    }

    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E>> {
        if range  == Range::Invalid {
            return Err(Error::InvalidRange);
        }
        let mut ctrl4 = self.read_register(Register::CTRL4)?;
        ctrl4 &= !0x30;
        ctrl4 |= range.bits() << 4;
        self.write_register(Register::CTRL4, ctrl4)
    }

    pub fn get_range(&mut self) -> Result<Range, Error<E>> {
        let ctrl4 = self.read_register(Register::CTRL4)?;
        Ok(Range::from((ctrl4 >> 4) & 0x03))
    }

    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }

    fn read_accel_bytes(&mut self) -> Result<[u8;6], Error<E>> {
        let mut data = [0u8;6];
        self.i2c
            .write_read(self.address, &[Register::OUT_X_L.addr() | 0x80], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data))
    }

    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }
        self.i2c.write(self.address, &[register.addr(), value]).map_err(Error::I2C)
    }

    pub fn try_into_tracker(mut self) -> Result<Tracker<Self, I16x3>, Error<E>> 
    where 
        E: Debug
    {
        self.set_range(Range::G8)?;
        Ok(Tracker::new(self, 3700))
    }
}

impl<I2C, E> Accelerometer<I16x3> for Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get acceleration reading from the accelerometer
    fn acceleration(&mut self) -> Result<I16x3, Error<E>> {
       let accel_bytes = self.read_accel_bytes()?;
       let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
       let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
       let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());
       Ok(I16x3::new(x, y, z))
    }
}
