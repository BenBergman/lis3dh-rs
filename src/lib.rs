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

use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};

use embedded_hal::blocking::i2c::{self, WriteRead};
use embedded_hal::blocking::spi::{self, Transfer};

use embedded_hal::digital::v2::OutputPin;

mod register;
use register::*;
pub use register::{DataRate, DataStatus, Mode, Range, SlaveAddr};

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<BusError, PinError> {
    /// I²C bus error
    Bus(BusError),
    Pin(PinError),

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

/// `LIS3DH` driver.
pub struct Lis3dh<CORE> {
    core: CORE,
}

impl<I2C, E> Lis3dh<Lis3dhI2C<I2C>>
where
    I2C: WriteRead<Error = E> + i2c::Write<Error = E>,
{
    /// Create a new LIS3DH driver from the given I2C peripheral.
    /// Default is Hz_400 HighResolution.
    pub fn new_i2c(
        i2c: I2C,
        address: SlaveAddr,
    ) -> Result<Self, Error<E, core::convert::Infallible>> {
        let core = Lis3dhI2C {
            i2c,
            address: address.addr(),
        };

        let mut lis3dh = Lis3dh { core };

        lis3dh.initialize()?;

        Ok(lis3dh)
    }
}

impl<SPI, NSS, ESPI, ENSS> Lis3dh<Lis3dhSPI<SPI, NSS>>
where
    SPI: spi::Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
{
    /// Create a new LIS3DH driver from the given SPI peripheral.
    pub fn new_spi(spi: SPI, nss: NSS) -> Result<Self, Error<ESPI, ENSS>> {
        let core = Lis3dhSPI { spi, nss };

        let mut lis3dh = Lis3dh { core };

        lis3dh.initialize()?;

        Ok(lis3dh)
    }
}

impl<CORE> Lis3dh<CORE>
where
    CORE: Lis3dhCore,
{
    fn initialize(&mut self) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        if self.get_device_id()? != DEVICE_ID {
            return Err(Error::WrongAddress);
        }

        // Block data update
        self.write_register(Register::CTRL4, BDU)?;

        self.set_mode(Mode::HighResolution)?;

        self.set_datarate(DataRate::Hz_400)?;

        self.enable_axis((true, true, true))?;

        // Enable ADCs.
        self.write_register(Register::TEMP_CFG, ADC_EN)?;

        Ok(())
    }
    /// `WHO_AM_I` register.
    pub fn get_device_id(&mut self) -> Result<u8, Error<CORE::BusError, CORE::PinError>> {
        self.read_register(Register::WHOAMI)
    }

    /// X,Y,Z-axis enable.
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    fn enable_axis(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
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
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
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
    pub fn get_mode(&mut self) -> Result<Mode, Error<CORE::BusError, CORE::PinError>> {
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
    pub fn set_datarate(
        &mut self,
        datarate: DataRate,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            // Mask off lowest 4 bits
            ctrl1 &= !ODR_MASK;
            // Write in new output data rate to highest 4 bits
            ctrl1 |= datarate.bits() << 4;

            ctrl1
        })
    }

    /// Read the current data selection rate.
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<CORE::BusError, CORE::PinError>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        let odr = (ctrl1 >> 4) & 0x0F;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Full-scale selection.
    pub fn set_range(&mut self, range: Range) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.modify_register(Register::CTRL4, |mut ctrl4| {
            // Mask off lowest 4 bits
            ctrl4 &= !FS_MASK;
            // Write in new full-scale to highest 4 bits
            ctrl4 |= range.bits() << 4;

            ctrl4
        })
    }

    /// Read the current full-scale.
    pub fn get_range(&mut self) -> Result<Range, Error<CORE::BusError, CORE::PinError>> {
        let ctrl4 = self.read_register(Register::CTRL4)?;
        let fs = (ctrl4 >> 4) & 0x03;

        Range::try_from(fs).map_err(|_| Error::InvalidRange)
    }

    /// Set `REFERENCE` register.
    pub fn set_ref(&mut self, reference: u8) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(Register::REFERENCE, reference)
    }

    /// Read the `REFERENCE` register.
    pub fn get_ref(&mut self) -> Result<u8, Error<CORE::BusError, CORE::PinError>> {
        self.read_register(Register::REFERENCE)
    }

    /// Accelerometer data-available status.
    pub fn get_status(&mut self) -> Result<DataStatus, Error<CORE::BusError, CORE::PinError>> {
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
    pub fn is_data_ready(&mut self) -> Result<bool, Error<CORE::BusError, CORE::PinError>> {
        let value = self.get_status()?;

        Ok(value.zyxda)
    }

    /// Temperature sensor enable.
    /// `TEMP_CGF_REG`: `TEMP_EN`, the BDU bit in `CTRL_REG4` is also set.
    pub fn enable_temp(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.register_xset_bits(Register::TEMP_CFG, ADC_EN & TEMP_EN, enable)?;

        // enable block data update (required for temp reading)
        if enable {
            self.register_xset_bits(Register::CTRL4, BDU, true)?;
        }

        Ok(())
    }

    /// Raw temperature sensor data as `i16`. The temperature sensor __must__
    /// be enabled via `enable_temp` prior to reading.
    pub fn get_temp_out(&mut self) -> Result<i16, Error<CORE::BusError, CORE::PinError>> {
        let out_l = self.read_register(Register::OUT_ADC3_L)?;
        let out_h = self.read_register(Register::OUT_ADC3_H)?;

        Ok(i16::from_le_bytes([out_l, out_h]))
    }

    /// Temperature sensor data converted to `f32`. Output is in degree
    /// celsius. The temperature sensor __must__ be enabled via `enable_temp`
    /// prior to reading.
    pub fn get_temp_outf(&mut self) -> Result<f32, Error<CORE::BusError, CORE::PinError>> {
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
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>>
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
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.modify_register(reg, |v| v & !bits)
    }

    /// Set the given bits in the given register.
    fn register_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.modify_register(reg, |v| v | bits)
    }

    /// Set or clear the given given bits in the given register, depending on
    /// the value of `set`.
    fn register_xset_bits(
        &mut self,
        reg: Register,
        bits: u8,
        set: bool,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        if set {
            self.register_set_bits(reg, bits)
        } else {
            self.register_clear_bits(reg, bits)
        }
    }
}

impl<CORE> Accelerometer for Lis3dh<CORE>
where
    CORE: Lis3dhCore,
    CORE::PinError: Debug,
    CORE::BusError: Debug,
{
    type Error = Error<CORE::BusError, CORE::PinError>;

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

impl<CORE> RawAccelerometer<I16x3> for Lis3dh<CORE>
where
    CORE: Lis3dhCore,
    CORE::PinError: Debug,
    CORE::BusError: Debug,
{
    type Error = Error<CORE::BusError, CORE::PinError>;

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

pub trait Lis3dhCore {
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
}

impl<CORE> Lis3dhCore for Lis3dh<CORE>
where
    CORE: Lis3dhCore,
{
    type BusError = CORE::BusError;
    type PinError = CORE::PinError;

    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.core.write_register(register, value)
    }

    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        self.core.read_register(register)
    }

    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>> {
        self.core.read_accel_bytes()
    }
}

struct Lis3dhI2C<I2C> {
    /// Underlying I²C device
    i2c: I2C,

    /// Current I²C slave address
    address: u8,
}

impl<I2C, E> Lis3dhCore for Lis3dhI2C<I2C>
where
    I2C: WriteRead<Error = E> + i2c::Write<Error = E>,
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

struct Lis3dhSPI<SPI, NSS> {
    /// Underlying SPI device
    spi: SPI,

    nss: NSS,
}

impl<SPI, NSS, ESPI, ENSS> Lis3dhSPI<SPI, NSS>
where
    SPI: spi::Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
{
    /// turn on the SPI slave
    fn nss_turn_on(&mut self) -> Result<(), Error<ESPI, ENSS>> {
        self.nss.set_low().map_err(Error::Pin)
    }

    /// turn off the SPI slave
    fn nss_turn_off(&mut self) -> Result<(), Error<ESPI, ENSS>> {
        self.nss.set_high().map_err(Error::Pin)
    }

    /// Writes to many registers. Does not check whether all registers
    /// can be written to
    unsafe fn write_multiple_regs(
        &mut self,
        start_register: Register,
        data: &[u8],
    ) -> Result<(), Error<ESPI, ENSS>> {
        self.nss_turn_on()?;
        let res = self
            .spi
            .write(&[start_register.addr() | 0x40])
            .and_then(|_| self.spi.write(data))
            .map_err(Error::Bus);
        self.nss_turn_off()?;
        res
    }

    /// Read from the registers for each of the 3 axes.
    fn read_multiple_regs(
        &mut self,
        start_register: Register,
        buf: &mut [u8],
    ) -> Result<(), Error<ESPI, ENSS>> {
        self.nss_turn_on()?;
        self.spi
            .write(&[start_register.addr() | 0xC0])
            .and_then(|_| self.spi.transfer(buf))
            .map_err(Error::Bus)
            .map(|_| self.nss_turn_off())?
    }
}

impl<SPI, NSS, ESPI, ENSS> Lis3dhCore for Lis3dhSPI<SPI, NSS>
where
    SPI: spi::Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
{
    type BusError = ESPI;
    type PinError = ENSS;

    /// Read from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<ESPI, ENSS>> {
        let mut data = [0u8; 6];
        self.read_multiple_regs(Register::OUT_X_L, &mut data)?;
        Ok(data)
    }

    /// Write a byte to the given register.
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<ESPI, ENSS>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }
        unsafe { self.write_multiple_regs(register, &[value]) }
    }

    /// Read a byte from the given register.
    fn read_register(&mut self, register: Register) -> Result<u8, Error<ESPI, ENSS>> {
        let mut data = [0];

        self.nss_turn_on()?;
        self.spi
            .write(&[register.addr() | 0x80])
            .and_then(|_| self.spi.transfer(&mut data))
            .map_err(Error::Bus)
            .map(|_| self.nss_turn_off())??;
        Ok(data[0])
    }
}
