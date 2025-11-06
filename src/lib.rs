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

mod interrupts;
mod register;

use interrupts::*;
pub use interrupts::{
    Detect4D, Interrupt1, Interrupt2, InterruptConfig, InterruptMode, InterruptSource, IrqPin,
    IrqPin1Config, IrqPin2Config, LatchInterruptRequest,
};

use register::*;
pub use register::{
    ClickCount, DataRate, DataStatus, Duration, FifoMode, FifoStatus, HighPassFilterConfig,
    HighPassFilterCutoff, HighPassFilterMode, Mode, Range, Register, SlaveAddr, Threshold,
};

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
    /// An example using the [nrf52840_hal](https://docs.rs/nrf52840-hal/latest/nrf52840_hal/index.html):
    ///
    ///     use nrf52840_hal::gpio::{Level, PushPull};
    ///     use lis3dh::Lis3dh;
    ///
    ///     let peripherals = nrf52840_hal::pac::Peripherals::take().unwrap();
    ///     let pins = p0::Parts::new(peripherals.P0);
    ///
    ///     let twim0_scl = pins.p0_31.into_floating_input().degrade();
    ///     let twim0_sda = pins.p0_30.into_floating_input().degrade();
    ///
    ///     let i2c = nrf52840_hal::twim::Twim::new(
    ///         peripherals.TWIM0,
    ///         nrf52840_hal::twim::Pins {
    ///             scl: twim0_scl,
    ///             sda: twim0_sda,
    ///         },
    ///         nrf52840_hal::twim::Frequency::K400,
    ///     );
    ///
    ///     let lis3dh = Lis3dh::new_i2c(i2c, lis3dh::SlaveAddr::Default).unwrap();
    pub fn new_i2c(
        i2c: I2C,
        address: SlaveAddr,
    ) -> Result<Self, Error<E, core::convert::Infallible>> {
        Self::new_i2c_with_config(i2c, address, Configuration::default())
    }

    pub fn new_i2c_with_config(
        i2c: I2C,
        address: SlaveAddr,
        config: Configuration,
    ) -> Result<Self, Error<E, core::convert::Infallible>> {
        let core = Lis3dhI2C {
            i2c,
            address: address.addr(),
        };

        let mut lis3dh = Lis3dh { core };

        lis3dh.configure(config)?;

        Ok(lis3dh)
    }
}

impl<SPI, NSS, ESPI, ENSS> Lis3dh<Lis3dhSPI<SPI, NSS>>
where
    SPI: spi::Write<u8, Error = ESPI> + Transfer<u8, Error = ESPI>,
    NSS: OutputPin<Error = ENSS>,
{
    /// Create a new LIS3DH driver from the given SPI peripheral.
    /// An example using the [nrf52840_hal](https://docs.rs/nrf52840-hal/latest/nrf52840_hal/index.html):
    ///
    ///     use nrf52840_hal::gpio::{p0::{Parts, P0_28}, *};
    ///     use nrf52840_hal::spim::Spim;
    ///     use lis3dh::Lis3dh;
    ///
    ///     let peripherals = nrf52840_hal::pac::Peripherals::take().unwrap();
    ///     let port0 = Parts::new(peripherals.P0);
    ///
    ///     // define the chip select pin
    ///     let cs: P0_28<Output<PushPull>> = port0.p0_28.into_push_pull_output(Level::High);
    ///
    ///     // spi pins: clock, miso, mosi
    ///     let pins = nrf52840_hal::spim::Pins {
    ///         sck: port0.p0_31.into_push_pull_output(Level::Low).degrade(),
    ///         miso: Some(port0.p0_30.into_push_pull_output(Level::Low).degrade()),
    ///         mosi: Some(port0.p0_29.into_floating_input().degrade()),
    ///     };
    ///
    ///     // set up the spi peripheral
    ///     let spi = Spim::new(
    ///         peripherals.SPIM2,
    ///         pins,
    ///         nrf52840_hal::spim::Frequency::K500,
    ///         nrf52840_hal::spim::MODE_0,
    ///         0,
    ///     );
    ///
    ///     // create and initialize the sensor
    ///     let lis3dh = Lis3dh::new_spi(spi, cs).unwrap();
    pub fn new_spi(spi: SPI, nss: NSS) -> Result<Self, Error<ESPI, ENSS>> {
        Self::new_spi_with_config(spi, nss, Configuration::default())
    }

    pub fn new_spi_with_config(
        spi: SPI,
        nss: NSS,
        config: Configuration,
    ) -> Result<Self, Error<ESPI, ENSS>> {
        let core = Lis3dhSPI { spi, nss };

        let mut lis3dh = Lis3dh { core };

        lis3dh.configure(config)?;

        Ok(lis3dh)
    }
}

impl<CORE> Lis3dh<CORE>
where
    CORE: Lis3dhCore,
{
    /// Configure the device
    pub fn configure(
        &mut self,
        conf: Configuration,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
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
        let fs = (ctrl4 >> 4) & 0b0011;

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

    /// Configure click threshold.
    ///
    /// lir_click: If this bit is set, the interrupt is
    /// kept high until the CLICK_SRC register is read. Otherwise, the
    /// interrupt is high for the duration of the click event.
    ///
    /// threshold: The threshold for click detection from 0-127, in
    /// increments of (full scale (in G's) / 128).
    pub fn set_click_threshold(
        &mut self,
        lir_click: bool,
        threshold: u8,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        let value = if lir_click {
            LIR_CLICK | threshold
        } else {
            !LIR_CLICK & threshold
        };
        self.write_register(Register::CLICK_THS, value)
    }

    /// Set click time limit.
    ///
    /// time_limit: The time interval between the start and end of a click
    /// event, from 0-255, in increments of (1/ODR) e.g 2.5ms at 400Hz.
    pub fn set_click_time_limit(
        &mut self,
        time_limit: u8,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(Register::TIME_LIMIT, time_limit)
    }

    /// Enable single- or double-click detection.
    pub fn enable_xyz_click_detection(
        &mut self,
        count: ClickCount,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(Register::CLICK_CFG, count as u8)
    }

    /// Click detection source.
    fn get_click_src(&mut self) -> Result<ClickSrc, Error<CORE::BusError, CORE::PinError>> {
        let click_src = self.read_register(Register::CLICK_SRC)?;

        Ok(ClickSrc {
            ia: (click_src & IA) != 0,
            sign: (click_src & SIGN) != 0,
            sclick: (click_src & SCLICK) != 0,
            dclick: (click_src & DCLICK) != 0,
            z: (click_src & Z) != 0,
            y: (click_src & Y) != 0,
            x: (click_src & X) != 0,
        })
    }

    /// Report number of clicks detected.
    pub fn click_count(&mut self) -> Result<usize, Error<CORE::BusError, CORE::PinError>> {
        let click_src = self.get_click_src()?;

        if click_src.sclick {
            Ok(1)
        } else if click_src.dclick {
            Ok(2)
        } else {
            Ok(0)
        }
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

    /// Clear the given bits in the given register. For example:
    ///
    ///     lis3dh.register_clear_bits(0b0110)
    ///
    /// This call clears (sets to 0) the bits at index 1 and 2. Other bits of the register are not touched.
    pub fn register_clear_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.modify_register(reg, |v| v & !bits)
    }

    /// Set the given bits in the given register. For example:
    ///
    ///     lis3dh.register_set_bits(0b0110)
    ///
    /// This call sets to 1 the bits at index 1 and 2. Other bits of the register are not touched.
    pub fn register_set_bits(
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

    /// Configure one of the interrupt pins
    ///
    ///     lis3dh.configure_interrupt_pin(IrqPin1Config {
    ///         // Raise if interrupt 1 is raised
    ///         ia1_en: true,
    ///         // Disable for all other interrupts
    ///         ..IrqPin1Config::default()
    ///     })?;
    pub fn configure_interrupt_pin<P: IrqPin>(
        &mut self,
        pin: P,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(P::ctrl_reg(), pin.bits())
    }

    /// Configure an IRQ source
    ///
    /// Example: configure interrupt 1 to fire when there is movement along any of the axes.
    ///
    ///     lis3dh.configure_irq_src(
    ///         lis3dh::Interrupt1,
    ///         lis3dh::InterruptMode::Movement,
    ///         lis3dh::InterruptConfig::high_and_low(),
    ///     )?;
    pub fn configure_irq_src<I: Interrupt>(
        &mut self,
        int: I,
        interrupt_mode: InterruptMode,
        interrupt_config: InterruptConfig,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.configure_irq_src_and_control(
            int,
            interrupt_mode,
            interrupt_config,
            LatchInterruptRequest::default(),
            Detect4D::default(),
        )
    }

    /// Configure an IRQ source.
    ///
    /// LIS (latch interrupt request) will latch (keep active) the interrupt until the [`Lis3dh::get_irq_src`] is read.
    ///
    /// 4D detection is a subset of the 6D detection where detection on the Z axis is disabled.
    /// This setting only has effect when the interrupt mode is either `Movement` or `Position`.
    ///
    /// Example: configure interrupt 1 to fire when there is movement along any of the axes.
    ///
    ///     lis3dh.configure_irq_src(
    ///         lis3dh::Interrupt1,
    ///         lis3dh::InterruptMode::Movement,
    ///         lis3dh::InterruptConfig::high_and_low(),
    ///         lis3dh::LatchInterruptRequest::Enable,
    ///         lis3dh::Detect4D::Enable,
    ///     )?;
    pub fn configure_irq_src_and_control<I: Interrupt>(
        &mut self,
        _int: I,
        interrupt_mode: InterruptMode,
        interrupt_config: InterruptConfig,
        latch_interrupt_request: LatchInterruptRequest,
        detect_4d: Detect4D,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        let latch_interrupt_request =
            matches!(latch_interrupt_request, LatchInterruptRequest::Enable);

        let detect_4d = matches!(detect_4d, Detect4D::Enable);

        if latch_interrupt_request || detect_4d {
            let latch = (latch_interrupt_request as u8) << I::lir_int_bit();
            let d4d = (detect_4d as u8) << I::d4d_int_bit();
            self.register_set_bits(Register::CTRL5, latch | d4d)?;
        }
        self.write_register(I::cfg_reg(), interrupt_config.to_bits(interrupt_mode))
    }

    /// Set the minimum duration for the Interrupt event to be recognized.
    ///
    /// Example: the event has to last at least 25 miliseconds to be recognized.
    ///
    ///     // let mut lis3dh = ...
    ///     let duration = Duration::miliseconds(DataRate::Hz_400, 25.0);
    ///     lis3dh.configure_irq_duration(duration);
    #[doc(alias = "INT1_DURATION")]
    #[doc(alias = "INT2_DURATION")]
    pub fn configure_irq_duration<I: Interrupt>(
        &mut self,
        _int: I,
        duration: Duration,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(I::duration_reg(), duration.0)
    }

    /// Set the minimum magnitude for the Interrupt event to be recognized.
    ///
    /// Example: the event has to have a magnitude of at least 1.1g to be recognized.
    ///
    ///     // let mut lis3dh = ...
    ///     let threshold = Threshold::g(Range::G2, 1.1);
    ///     lis3dh.configure_irq_threshold(threshold);
    #[doc(alias = "INT1_THS")]
    #[doc(alias = "INT2_THS")]
    pub fn configure_irq_threshold<I: Interrupt>(
        &mut self,
        _int: I,
        threshold: Threshold,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(I::ths_reg(), threshold.0)
    }

    /// Get interrupt source. The `interrupt_active` field is true when an interrupt is active.
    /// The other fields specify what measurement caused the interrupt.
    pub fn get_irq_src<I: Interrupt>(
        &mut self,
        _int: I,
    ) -> Result<InterruptSource, Error<CORE::BusError, CORE::PinError>> {
        let irq_src = self.read_register(I::src_reg())?;
        Ok(InterruptSource::from_bits(irq_src))
    }

    /// Configure high-pass filter.
    ///
    /// The high-pass filter can be applied to interrupt paths and/or data output.
    /// This is useful for motion detection where you want to remove the DC component (gravity).
    ///
    /// Example: enable high-pass filter for interrupt 1 only, keep data output unfiltered.
    ///
    ///     lis3dh.configure_high_pass_filter(HighPassFilterConfig {
    ///         enable_for_interrupt1: true,
    ///         ..Default::default()
    ///     })?;
    #[doc(alias = "CTRL_REG2")]
    #[doc(alias = "HPF")]
    pub fn configure_high_pass_filter(
        &mut self,
        config: HighPassFilterConfig,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        let mut ctrl2 = 0u8;

        ctrl2 |= (config.mode as u8) << 6;
        ctrl2 |= (config.cutoff as u8) << 4;

        if config.enable_for_data {
            ctrl2 |= FDS;
        }
        if config.enable_for_click {
            ctrl2 |= HPCLICK;
        }
        if config.enable_for_interrupt2 {
            ctrl2 |= HPIS2;
        }
        if config.enable_for_interrupt1 {
            ctrl2 |= HPIS1;
        }

        self.write_register(Register::CTRL2, ctrl2)
    }

    /// Read current high-pass filter configuration.
    #[doc(alias = "CTRL_REG2")]
    pub fn get_high_pass_filter_config(
        &mut self,
    ) -> Result<HighPassFilterConfig, Error<CORE::BusError, CORE::PinError>> {
        let ctrl2 = self.read_register(Register::CTRL2)?;

        Ok(HighPassFilterConfig {
            mode: match (ctrl2 >> 6) & 0b11 {
                0b00 => HighPassFilterMode::Normal,
                0b01 => HighPassFilterMode::Reference,
                0b10 => HighPassFilterMode::AutoresetOnInterrupt,
                _ => HighPassFilterMode::Normal,
            },
            cutoff: match (ctrl2 >> 4) & 0b11 {
                0b00 => HighPassFilterCutoff::Lowest,
                0b01 => HighPassFilterCutoff::Low,
                0b10 => HighPassFilterCutoff::Medium,
                0b11 => HighPassFilterCutoff::High,
                _ => HighPassFilterCutoff::Lowest,
            },
            enable_for_interrupt1: ctrl2 & HPIS1 != 0,
            enable_for_interrupt2: ctrl2 & HPIS2 != 0,
            enable_for_click: ctrl2 & HPCLICK != 0,
            enable_for_data: ctrl2 & FDS != 0,
        })
    }

    /// Configure 'Sleep to wake' and 'Return to sleep' threshold and duration.
    ///
    /// The LIS3DH can be programmed to automatically switch to low-power mode upon recognition of a determined event.
    /// Once the event condition is over, the device returns back to the preset normal or highresolution mode.
    ///
    /// Example: enter low-power mode. When a measurement above 1.1g is registered, then wake up
    /// for 25ms to send the data.
    ///
    ///     // let mut lis3dh = ...
    ///
    ///     let range = Range::default();
    ///     let data_rate = DataRate::Hz_400;
    ///
    ///     let threshold = Threshold::g(range, 1.1);
    ///     let duration = Duration::miliseconds(data_rate, 25.0);
    ///
    ///     lis3dh.configure_switch_to_low_power(threshold, duration)?;
    ///
    ///     lis3dh.set_datarate(data_rate)?;
    #[doc(alias = "ACT_THS")]
    #[doc(alias = "ACT_DUR")]
    #[doc(alias = "act")]
    pub fn configure_switch_to_low_power(
        &mut self,
        threshold: Threshold,
        duration: Duration,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(Register::ACT_THS, threshold.0 & 0b0111_1111)?;
        self.write_register(Register::ACT_DUR, duration.0)
    }

    /// Reboot memory content
    pub fn reboot_memory_content(&mut self) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.register_set_bits(Register::CTRL5, 0b1000_0000)
    }

    const FIFO_ENABLE_BIT: u8 = 0b0100_0000;

    /// Configures FIFO and then enables it
    pub fn enable_fifo(
        &mut self,
        mode: FifoMode,
        threshold: u8,
    ) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        debug_assert!(threshold <= 0b0001_1111);

        let bits = (threshold & 0b0001_1111) | mode.to_bits();
        self.write_register(Register::FIFO_CTRL, bits)?;
        self.register_set_bits(Register::CTRL5, Self::FIFO_ENABLE_BIT)
    }

    /// Disable FIFO. This resets the FIFO state
    pub fn disable_fifo(&mut self) -> Result<(), Error<CORE::BusError, CORE::PinError>> {
        self.write_register(Register::FIFO_CTRL, 0x00)?;
        self.register_clear_bits(Register::CTRL5, Self::FIFO_ENABLE_BIT)
    }

    /// Get the status of the FIFO
    pub fn get_fifo_status(&mut self) -> Result<FifoStatus, Error<CORE::BusError, CORE::PinError>> {
        let status = self.read_register(Register::FIFO_SRC)?;

        Ok(FifoStatus::from_bits(status))
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

/// Marker to indicate I2C is used to communicate with the Lis3dh
pub struct Lis3dhI2C<I2C> {
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

/// Marker to indicate SPI is used to communicate with the Lis3dh
pub struct Lis3dhSPI<SPI, NSS> {
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
            .map_err(Error::Bus)?;
        self.nss_turn_off()
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
            .map_err(Error::Bus)?;
        self.nss_turn_off()?;
        Ok(data[0])
    }
}

/// Sensor configuration options
#[derive(Debug, Clone, Copy)]
pub struct Configuration {
    /// The operating mode, default [`Mode::HighResolution`].
    pub mode: Mode,
    /// The output data rate, default [`DataRate::Hz_400`].
    pub datarate: DataRate,
    /// Measure changes in the x axis, default `true`.
    pub enable_x_axis: bool,
    /// Measure changes in the y axis, default `true`.
    pub enable_y_axis: bool,
    /// Measure changes in the z axis, default `true`.
    pub enable_z_axis: bool,
    /// When is data updated
    ///
    /// - when `true`: only after data is read
    /// - when `false`: continually
    ///
    /// default `true`
    pub block_data_update: bool,
    /// Enable temperature measurements. When set, it implies `block_data_update = true`.
    ///
    /// default: `false`
    pub enable_temperature: bool,
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
