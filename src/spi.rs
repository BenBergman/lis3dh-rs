use core::fmt::Debug;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};

use crate::{register::*, Configuration, Error, Lis3dhImpl};

/// `LIS3DH` driver.
pub struct Lis3dh<SPI, NSS> {
    /// Underlying SPI device
    spi: SPI,
    /// Active-low slave-select pin
    nss: NSS,
}

impl<SPI, NSS, ESPI, ENSS> Lis3dh<SPI, NSS>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    ESPI: Debug,
    ENSS: Debug,
{
    /// Create a new LIS3DH driver from the given SPI peripheral.
    pub fn new(spi: SPI, nss: NSS) -> Result<Self, Error<ESPI, ENSS>> {
        Self::with_config(spi, nss, Default::default())
    }

    pub fn with_config(
        spi: SPI,
        nss: NSS,
        config: Configuration,
    ) -> Result<Self, Error<ESPI, ENSS>> {
        let mut lis3dh = Lis3dh { spi, nss };

        lis3dh.initialize_with_config(config)?;

        Ok(lis3dh)
    }

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

impl<SPI, NSS, ESPI, ENSS> crate::Lis3dhImpl for Lis3dh<SPI, NSS>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    ESPI: Debug,
    ENSS: Debug,
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

impl<SPI, NSS, ESPI, ENSS> RawAccelerometer<I16x3> for Lis3dh<SPI, NSS>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    ESPI: Debug,
    ENSS: Debug,
{
    type Error = Error<ESPI, ENSS>;

    /// Get raw acceleration data from the accelerometer.
    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let (x, y, z) = self.acceleration_raw()?;

        Ok(I16x3::new(x, y, z))
    }
}

impl<SPI, NSS, ESPI, ENSS> Accelerometer for Lis3dh<SPI, NSS>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    ESPI: Debug,
    ENSS: Debug,
{
    type Error = Error<ESPI, ENSS>;

    /// Get normalized ±g reading from the accelerometer.
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let (x, y, z) = self.normalize_acceleration()?;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
    }
}
