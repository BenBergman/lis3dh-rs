//! Platform-agnostic LIS3DH accelerometer driver which uses I²C via
//! [embedded-hal]. This driver implements the [`Accelerometer`][acc-trait]
//! and [`RawAccelerometer`][raw-trait] traits from the `accelerometer` crate.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [acc-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//! [raw-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.RawAccelerometer.html
//!

#![no_std]

use core::convert::TryInto;
use core::fmt::Debug;

use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer, Tracker};
use embedded_hal::blocking::i2c::{Write, WriteRead};

mod register;
pub use register::*;

#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data.
    WrongAddress,
    WriteToReadOnly,
    InvalidDataRate,
    InvalidMode,
    InvalidRange,
}

/// `LIS3DH` driver
pub struct Lis3dh<I2C> {
    /// Underlying I²C device
    i2c: I2C,

    /// Current I²C slave address
    address: u8,
}

impl<I2C, E> Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    /// Create a new LIS3DH driver from the given I2C peripheral
    pub fn new(i2c: I2C, address: SlaveAddr) -> Result<Self, Error<E>> {
        let mut lis3dh = Lis3dh {
            i2c,
            address: address.addr(),
        };

        if lis3dh.get_device_id()? != DEVICE_ID {
            return Err(Error::WrongAddress);
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

    /// `WHO_AM_I` register
    pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::WHOAMI)
    }

    /// Operating mode selection.
    /// `CTRL_REG1`: `LPen` bit, `CTRL_REG4`: `HR` bit
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::LowPower => {
                self.register_set_bits(Register::CTRL1, 0x8)?;
                self.register_reset_bits(Register::CTRL4, 0x8)?;
            }
            Mode::Normal => {
                self.register_reset_bits(Register::CTRL1, 0x8)?;
                self.register_reset_bits(Register::CTRL4, 0x8)?;
            }
            Mode::HighResolution => {
                self.register_reset_bits(Register::CTRL1, 0x8)?;
                self.register_set_bits(Register::CTRL4, 0x8)?;
            }
        }

        Ok(())
    }

    /// Read the current operating mode
    pub fn get_mode(&mut self) -> Result<Mode, Error<E>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        let ctrl4 = self.read_register(Register::CTRL4)?;

        let lp = (ctrl1 >> 3) & 0x1 != 0;
        let hr = (ctrl4 >> 3) & 0x1 != 0;

        let mode = match (lp, hr) {
            (true, false) => Mode::LowPower,
            (false, false) => Mode::Normal,
            (false, true) => Mode::HighResolution,
            _ => return Err(Error::InvalidMode)
        };

        Ok(mode)
    }

    /// Data rate selection
    pub fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<E>> {
        if datarate == DataRate::Invalid {
            return Err(Error::InvalidDataRate);
        }

        self.modify_register(Register::CTRL1, |mut ctrl1| {
            // Mask off lowest 4 bits
            ctrl1 &= 0xF;
            // Write in new data rate to highest 4 bits
        ctrl1 |= datarate.bits() << 4;

            ctrl1
        })
    }

    /// Read the current data selection rate
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;

        Ok(DataRate::from((ctrl1 >> 4) & 0x0F))
    }

    /// Range selection
    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E>> {
        if range == Range::Invalid {
            return Err(Error::InvalidRange);
        }

        self.modify_register(Register::CTRL4, |mut ctrl4| {
            // Mask off lowest 4 bits
        ctrl4 &= !0x30;
            // Write in new range to highest 4 bits
        ctrl4 |= range.bits() << 4;

            ctrl4
        })
    }

    /// Read the current range
    pub fn get_range(&mut self) -> Result<Range, Error<E>> {
        let ctrl4 = self.read_register(Register::CTRL4)?;

        Ok(Range::from((ctrl4 >> 4) & 0x03))
    }

    /// Use this accelerometer as an orientation tracker
    pub fn try_into_tracker(&mut self) -> Result<Tracker, Error<E>> {
        self.set_range(Range::G8)?;
        Ok(Tracker::new(3700.0))
    }

    /// Modify a register's value
    fn modify_register<F>(&mut self, register: Register, f: F) -> Result<(), Error<E>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register)?;
        self.write_register(register, f(value))?;

        Ok(())
    }

    fn register_reset_bits(&mut self, reg: Register, bits: u8) -> Result<(), Error<E>> {
        self.modify_register(reg, |v| v & !bits)
    }

    fn register_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), Error<E>> {
        self.modify_register(reg, |v| v | bits)
    }

    /// Read from the registers for each of the 3 axes
    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<E>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(self.address, &[Register::OUT_X_L.addr() | 0x80], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data))
    }

    /// Write to the given register
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.address, &[register.addr(), value])
            .map_err(Error::I2C)
    }

    /// Read from the given register
    fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut data = [0];

        self.i2c
            .write_read(self.address, &[register.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }
}

impl<I2C, E> Accelerometer for Lis3dh<I2C>
    where 
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get normalized ±g reading from the accelerometer
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let range = self.get_range()?;
        if range == Range::Invalid {
            return Err(AccelerometerError::from(Self::Error::InvalidRange));
        }

        let acc_raw: I16x3 = self.accel_raw()?;
        let sensitivity = match range {
            Range::G16 => 0.012,
            Range::G8 => 0.004,
            Range::G4 => 0.002,
            Range::G2 => 0.001,
            _ => unreachable!(),
        };

        Ok(F32x3::new(
            (acc_raw.x >> 4) as f32 * sensitivity,
            (acc_raw.y >> 4) as f32 * sensitivity,
            (acc_raw.z >> 4) as f32 * sensitivity,
        ))
    }

    /// Get the sample rate of the accelerometer data
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        let sample_rate = match self.get_datarate()? {
            DataRate::Hz_1344_LP5k => {
                if self.get_mode()? == Mode::LowPower {
                    5376.0
                } else {
                    1344.0
                }
            }
            DataRate::Hz_400 => 400.0,
            DataRate::Hz_200 => 200.0,
            DataRate::Hz_100 => 100.0,
            DataRate::Hz_50 => 50.0,
            DataRate::Hz_25 => 25.0,
            DataRate::Hz_10 => 10.0,
            DataRate::Hz_1 => 1.0,
            DataRate::PowerDown => 0.0,
            DataRate::LowPower_1K6HZ => 1600.0,
            DataRate::Invalid => 0.0,
        };

        Ok(sample_rate)
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get raw acceleration data from the accelerometer.
    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let accel_bytes = self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}
