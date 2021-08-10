//! Platform-agnostic LIS3DH accelerometer driver which uses I²C via
//! [embedded-hal]. This driver implements the [`Accelerometer`][acc-trait]
//! and [`RawAccelerometer`][raw-trait] traits from the [accelerometer] crate.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [accelerometer]: https://docs.rs/accelerometer
//! [acc-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//! [raw-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.RawAccelerometer.html
//!

#![no_std]

use core::convert::TryFrom;
use core::fmt::Debug;

pub use accelerometer;

mod register;
use register::*;
pub use register::{DataRate, DataStatus, Mode, Range, SlaveAddr};

pub mod i2c;
pub mod spi;

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<EBUS, EPIN> {
    /// bus (I²C or SPI) error
    Bus(EBUS),

    /// Pin error. Only used in SPI mode
    Pin(EPIN),

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid operating mode selection
    InvalidMode,

    /// Invalid full-scale selection
    InvalidRange,

    /// Attempted to write to a read-only register
    WriteToReadOnly,

    /// Invalid address provided
    WrongAddress,
}

pub trait Lis3dhImpl {
    type BusError;
    type PinError;

    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>>;

    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>>;

    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>>;

    /// Initalize the device given the configuration
    fn initialize_with_config(
        &mut self,
        conf: Configuration,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        if self.get_device_id()? != DEVICE_ID {
            return Err(Error::WrongAddress);
        }

        if conf.block_data_update || conf.enable_temperature {
            // Block data update
            self.write_register(Register::CTRL4, BDU)?;
        }

        self.set_mode(conf.mode)?;

        self.set_datarate(conf.datarate)?;

        self.enable_axis((conf.enable_x_axis, conf.enable_y_axis, conf.enable_z_axis))?;

        if conf.enable_temperature {
            self.enable_temp(true)?;
        }

        // Enable ADCs.
        self.write_register(Register::TEMP_CFG, ADC_EN)
    }

    /// `WHO_AM_I` register.
    fn get_device_id(&mut self) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        self.read_register(Register::WHOAMI)
    }

    /// X,Y,Z-axis enable.
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    fn enable_axis(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            ctrl1 &= !(X_EN | Y_EN | Z_EN); // disable all axes

            ctrl1 |= if x { X_EN } else { 0 };
            ctrl1 |= if y { Y_EN } else { 0 };
            ctrl1 |= if z { Z_EN } else { 0 };

            ctrl1
        })
    }

    /// Operating mode selection.  
    /// `CTRL_REG1`: `LPen` bit, `CTRL_REG4`: `HR` bit.  
    /// You need to wait for stabilization after setting. In future this
    /// function will be deprecated and instead take a `Delay` to do this for
    /// you.
    ///
    /// | From           | To             | Wait for   |
    /// |:---------------|:---------------|:-----------|
    /// | HighResolution | LowPower       | 1/datarate |
    /// | HighResolution | Normal         | 1/datarate |
    /// | Normal         | LowPower       | 1/datarate |
    /// | Normal         | HighResolution | 7/datarate |
    /// | LowPower       | Normal         | 1/datarate |
    /// | LowPower       | HighResolution | 7/datarate |
    fn set_mode(&mut self, mode: Mode) -> Result<(), Error<Self::BusError, Self::PinError>> {
        match mode {
            Mode::LowPower => {
                self.register_set_bits(Register::CTRL1, LP_EN)?;
                self.register_clear_bits(Register::CTRL4, HR)?;
            }
            Mode::Normal => {
                self.register_clear_bits(Register::CTRL1, LP_EN)?;
                self.register_clear_bits(Register::CTRL4, HR)?;
            }
            Mode::HighResolution => {
                self.register_clear_bits(Register::CTRL1, LP_EN)?;
                self.register_set_bits(Register::CTRL4, HR)?;
            }
        }

        Ok(())
    }

    /// Read the current operating mode.
    fn get_mode(&mut self) -> Result<Mode, Error<Self::BusError, Self::PinError>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        let ctrl4 = self.read_register(Register::CTRL4)?;

        let is_lp_set = (ctrl1 >> 3) & 0x01 != 0;
        let is_hr_set = (ctrl4 >> 3) & 0x01 != 0;

        let mode = match (is_lp_set, is_hr_set) {
            (true, false) => Mode::LowPower,
            (false, false) => Mode::Normal,
            (false, true) => Mode::HighResolution,
            _ => return Err(Error::InvalidMode),
        };

        Ok(mode)
    }

    /// Data rate selection.
    fn set_datarate(
        &mut self,
        datarate: DataRate,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            // Mask off lowest 4 bits
            ctrl1 &= !ODR_MASK;
            // Write in new output data rate to highest 4 bits
            ctrl1 |= datarate.bits() << 4;

            ctrl1
        })
    }

    /// Read the current data selection rate.
    fn get_datarate(&mut self) -> Result<DataRate, Error<Self::BusError, Self::PinError>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        let odr = (ctrl1 >> 4) & 0x0F;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Full-scale selection.
    fn set_range(&mut self, range: Range) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.modify_register(Register::CTRL4, |mut ctrl4| {
            // Mask off lowest 4 bits
            ctrl4 &= !FS_MASK;
            // Write in new full-scale to highest 4 bits
            ctrl4 |= range.bits() << 4;

            ctrl4
        })
    }

    /// Read the current full-scale.
    fn get_range(&mut self) -> Result<Range, Error<Self::BusError, Self::PinError>> {
        let ctrl4 = self.read_register(Register::CTRL4)?;
        let fs = (ctrl4 >> 4) & 0x03;

        Range::try_from(fs).map_err(|_| Error::InvalidRange)
    }

    /// Set `REFERENCE` register.
    fn set_ref(&mut self, reference: u8) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.write_register(Register::REFERENCE, reference)
    }

    /// Read the `REFERENCE` register.
    fn get_ref(&mut self) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        self.read_register(Register::REFERENCE)
    }

    /// Accelerometer data-available status.
    fn get_status(&mut self) -> Result<DataStatus, Error<Self::BusError, Self::PinError>> {
        let stat = self.read_register(Register::STATUS)?;

        Ok(DataStatus {
            zyxor: (stat & ZYXOR) != 0,
            xyzor: ((stat & XOR) != 0, (stat & YOR) != 0, (stat & ZOR) != 0),
            zyxda: (stat & ZYXDA) != 0,
            xyzda: ((stat & XDA) != 0, (stat & YDA) != 0, (stat & ZDA) != 0),
        })
    }

    /// Convenience function for `STATUS_REG` to confirm all three X, Y and
    /// Z-axis have new data available for reading by accel_raw and associated
    /// function calls.
    fn is_data_ready(&mut self) -> Result<bool, Error<Self::BusError, Self::PinError>> {
        let value = self.get_status()?;

        Ok(value.zyxda)
    }

    /// Temperature sensor enable.
    /// `TEMP_CGF_REG`: `TEMP_EN`, the BDU bit in `CTRL_REG4` is also set.
    fn enable_temp(&mut self, enable: bool) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.register_xset_bits(Register::TEMP_CFG, ADC_EN & TEMP_EN, enable)?;

        // enable block data update (required for temp reading)
        if enable {
            self.register_xset_bits(Register::CTRL4, BDU, true)?;
        }

        Ok(())
    }

    /// Raw temperature sensor data as `i16`. The temperature sensor __must__
    /// be enabled via `enable_temp` prior to reading.
    fn get_temp_out(&mut self) -> Result<i16, Error<Self::BusError, Self::PinError>> {
        let out_l = self.read_register(Register::OUT_ADC3_L)?;
        let out_h = self.read_register(Register::OUT_ADC3_H)?;

        Ok(i16::from_le_bytes([out_l, out_h]))
    }

    /// Temperature sensor data converted to `f32`. Output is in degree
    /// celsius. The temperature sensor __must__ be enabled via `enable_temp`
    /// prior to reading.
    fn get_temp_outf(&mut self) -> Result<f32, Error<Self::BusError, Self::PinError>> {
        let temp_out = self.get_temp_out()?;

        Ok(temp_out as f32 / 256.0 + 25.0)
    }

    /// Modify a register's value. Read the current value of the register,
    /// update the value with the provided function, and set the register to
    /// the return value.
    fn modify_register<F>(
        &mut self,
        register: Register,
        f: F,
    ) -> Result<(), Error<Self::BusError, Self::PinError>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register)?;

        self.write_register(register, f(value))
    }

    /// Clear the given bits in the given register.
    fn register_clear_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.modify_register(reg, |v| v & !bits)
    }

    /// Set the given bits in the given register.
    fn register_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.modify_register(reg, |v| v | bits)
    }

    /// Set or clear the given given bits in the given register, depending on
    /// the value of `set`.
    fn register_xset_bits(
        &mut self,
        reg: Register,
        bits: u8,
        set: bool,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        if set {
            self.register_set_bits(reg, bits)
        } else {
            self.register_clear_bits(reg, bits)
        }
    }
}

pub struct Configuration {
    /// When set, overrides block_data_update
    pub enable_temperature: bool,
    /// Whether data should only be updated after reading
    pub block_data_update: bool,
    pub mode: Mode,
    pub datarate: DataRate,
    pub enable_x_axis: bool,
    pub enable_y_axis: bool,
    pub enable_z_axis: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            enable_temperature: false,
            block_data_update: true,
            mode: Mode::HighResolution, // Question: should this be normal?
            datarate: DataRate::Hz_400,
            enable_x_axis: true,
            enable_y_axis: true,
            enable_z_axis: true,
        }
    }
}
