use core::{convert::TryInto, fmt::Debug};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};

use crate::{register::*, Configuration, Error, Lis3dhImpl};

/// `LIS3DH` driver.
pub struct Lis3dh<SPI, NSS, DELAY> {
    /// Underlying SPI device
    spi: SPI,
    /// Active-low slave-select pin
    nss: NSS,
    /// Blocking delay
    delay: DELAY,
}

impl<SPI, NSS, DELAY, ESPI, ENSS> Lis3dh<SPI, NSS, DELAY>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    DELAY: DelayUs<u8>,
    ESPI: Debug,
    ENSS: Debug,
{
    /// Create a new LIS3DH driver from the given SPI peripheral.
    pub fn new(spi: SPI, nss: NSS, delay: DELAY) -> Result<Self, Error<ESPI, ENSS>> {
        Self::with_config(spi, nss, delay, Default::default())
    }

    pub fn with_config(
        spi: SPI,
        nss: NSS,
        delay: DELAY,
        config: Configuration,
    ) -> Result<Self, Error<ESPI, ENSS>> {
        let mut lis3dh = Lis3dh { spi, nss, delay };

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
        self.delay.delay_us(1);
        let res = self
            .spi
            .write(&[start_register.addr() | 0x40])
            .and_then(|_| self.spi.write(data))
            .map_err(Error::Bus);
        self.delay.delay_us(1);
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
        self.delay.delay_us(1);
        self.spi
            .write(&[start_register.addr() | 0xC0])
            .and_then(|_| self.spi.transfer(buf))
            .map_err(Error::Bus)
            .map(|_| {
                self.delay.delay_us(1);
                self.nss_turn_off()
            })?
    }
}

impl<SPI, NSS, DELAY, ESPI, ENSS> crate::Lis3dhImpl for Lis3dh<SPI, NSS, DELAY>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    DELAY: DelayUs<u8>,
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
        self.delay.delay_us(1);
        self.spi
            .write(&[register.addr() | 0x80])
            .and_then(|_| self.spi.transfer(&mut data))
            .map_err(Error::Bus)
            .map(|_| {
                self.delay.delay_us(1);
                self.nss_turn_off()
            })??;
        Ok(data[0])
    }
}

impl<SPI, NSS, DELAY, ESPI, ENSS> RawAccelerometer<I16x3> for Lis3dh<SPI, NSS, DELAY>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    DELAY: DelayUs<u8>,
    ESPI: Debug,
    ENSS: Debug,
{
    type Error = Error<ESPI, ENSS>;

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

impl<SPI, NSS, DELAY, ESPI, ENSS> Accelerometer for Lis3dh<SPI, NSS, DELAY>
where
    SPI: Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
    DELAY: DelayUs<u8>,
    ESPI: Debug,
    ENSS: Debug,
{
    type Error = Error<ESPI, ENSS>;

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
