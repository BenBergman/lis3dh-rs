use core::convert::TryInto;
use core::fmt::Debug;

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};

use embedded_hal::blocking::i2c::{Write, WriteRead};

use crate::register::*;
use crate::{Configuration, Error, Lis3dhImpl};

/// `LIS3DH` driver.
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
    /// Create a new LIS3DH driver from the given I2C peripheral. Default is
    /// Hz_400 HighResolution.
    pub fn new(i2c: I2C, address: SlaveAddr) -> Result<Self, Error<E, core::convert::Infallible>> {
        Self::with_config(i2c, address, Default::default())
    }

    pub fn with_config(
        i2c: I2C,
        address: SlaveAddr,
        config: Configuration,
    ) -> Result<Self, Error<E, core::convert::Infallible>> {
        let mut lis3dh = Lis3dh {
            i2c,
            address: address.addr(),
        };

        lis3dh.initialize_with_config(config)?;

        Ok(lis3dh)
    }
}

impl<I2C, E> Lis3dhImpl for Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type BusError = E;
    type PinError = core::convert::Infallible;

    /// Read from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(self.address, &[Register::OUT_X_L.addr() | 0x80], &mut data)
            .map_err(Error::Bus)
            .and(Ok(data))
    }

    /// Write a byte to the given register.
    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.address, &[register.addr(), value])
            .map_err(Error::Bus)
    }

    /// Read a byte from the given register.
    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        let mut data = [0];

        self.i2c
            .write_read(self.address, &[register.addr()], &mut data)
            .map_err(Error::Bus)
            .and(Ok(data[0]))
    }
}

impl<I2C, E> Accelerometer for Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E, core::convert::Infallible>;

    /// Get normalized ±g reading from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        // The official driver from ST was used as a reference.
        // https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis3dh_STdC
        let mode = self.get_mode()?;
        let range = self.get_range()?;

        // See "2.1 Mechanical characteristics" in the datasheet to find the
        // values below. Scale values have all been divided by 1000 in order
        // to convert the resulting values from mG to G, while avoiding doing
        // any actual division on the hardware.
        let scale = match (mode, range) {
            // High Resolution mode
            (Mode::HighResolution, Range::G2) => 0.001,
            (Mode::HighResolution, Range::G4) => 0.002,
            (Mode::HighResolution, Range::G8) => 0.004,
            (Mode::HighResolution, Range::G16) => 0.012,
            // Normal mode
            (Mode::Normal, Range::G2) => 0.004,
            (Mode::Normal, Range::G4) => 0.008,
            (Mode::Normal, Range::G8) => 0.016,
            (Mode::Normal, Range::G16) => 0.048,
            // Low Power mode
            (Mode::LowPower, Range::G2) => 0.016,
            (Mode::LowPower, Range::G4) => 0.032,
            (Mode::LowPower, Range::G8) => 0.064,
            (Mode::LowPower, Range::G16) => 0.192,
        };

        // Depending on which Mode we are operating in, the data has different
        // resolution. Using this knowledge, we determine how many bits the
        // data needs to be shifted. This is necessary because the raw data
        // is in left-justified two's complement and we would like for it to be
        // right-justified instead.
        let shift: u8 = match mode {
            Mode::HighResolution => 4, // High Resolution:  12-bit
            Mode::Normal => 6,         // Normal:           10-bit
            Mode::LowPower => 8,       // Low Power:         8-bit
        };

        let acc_raw = self.accel_raw()?;
        let x = (acc_raw.x >> shift) as f32 * scale;
        let y = (acc_raw.y >> shift) as f32 * scale;
        let z = (acc_raw.z >> shift) as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for Lis3dh<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E, core::convert::Infallible>;

    /// Get raw acceleration data from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let accel_bytes = self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}
