use core::convert::TryInto;
use num_enum::TryFromPrimitive;

/// Possible I²C slave addresses.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SlaveAddr {
    /// Default slave address (`0x18`)
    Default = 0x18,

    /// Alternate slave address (`0x19`)
    Alternate = 0x19,
}

impl SlaveAddr {
    pub fn addr(self) -> u8 {
        self as u8
    }
}

/// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    STATUS_AUX = 0x07,
    OUT_ADC1_L = 0x08,
    OUT_ADC1_H = 0x09,
    OUT_ADC2_L = 0x0A,
    OUT_ADC2_H = 0x0B,
    OUT_ADC3_L = 0x0C,
    OUT_ADC3_H = 0x0D,
    WHOAMI = 0x0F,
    CTRL0 = 0x1E,
    TEMP_CFG = 0x1F,
    CTRL1 = 0x20,
    CTRL2 = 0x21,
    CTRL3 = 0x22,
    CTRL4 = 0x23,
    CTRL5 = 0x24,
    CTRL6 = 0x25,
    REFERENCE = 0x26,
    STATUS = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL = 0x2E,
    FIFO_SRC = 0x2F,
    INT1_CFG = 0x30,
    INT1_SRC = 0x31,
    INT1_THS = 0x32,
    INT1_DURATION = 0x33,
    INT2_CFG = 0x34,
    INT2_SRC = 0x35,
    INT2_THS = 0x36,
    INT2_DURATION = 0x37,
    CLICK_CFG = 0x38,
    CLICK_SRC = 0x39,
    CLICK_THS = 0x3A,
    TIME_LIMIT = 0x3B,
    TIME_LATENCY = 0x3C,
    TIME_WINDOW = 0x3D,
    ACT_THS = 0x3E,
    ACT_DUR = 0x3F,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Is the register read-only?
    pub fn read_only(self) -> bool {
        matches!(
            self,
            Register::STATUS_AUX
                | Register::OUT_ADC1_L
                | Register::OUT_ADC1_H
                | Register::OUT_ADC2_L
                | Register::OUT_ADC2_H
                | Register::OUT_ADC3_L
                | Register::OUT_ADC3_H
                | Register::WHOAMI
                | Register::STATUS
                | Register::OUT_X_L
                | Register::OUT_X_H
                | Register::OUT_Y_L
                | Register::OUT_Y_H
                | Register::OUT_Z_L
                | Register::OUT_Z_H
                | Register::FIFO_SRC
                | Register::INT1_SRC
                | Register::INT2_SRC
                | Register::CLICK_SRC
        )
    }
}

/// Full-scale selection.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Range {
    /// ±16g
    G16 = 0b11,

    /// ±8g
    G8 = 0b10,

    /// ±4g
    G4 = 0b01,

    /// ±2g (Default)
    G2 = 0b00,
}

impl Range {
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Convert the range into an value in mili-g
    pub const fn as_mg(self) -> u8 {
        match self {
            Range::G16 => 186,
            Range::G8 => 62,
            Range::G4 => 32,
            Range::G2 => 16,
        }
    }
}

impl Default for Range {
    fn default() -> Self {
        Range::G2
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Threshold(pub(crate) u8);

impl Threshold {
    /// Convert a value in multiples of the `g` constant (roughly 9.81) to a threshold.
    ///
    ///     assert_eq!(Threshold::g(Range::G2, 1.1), 69);
    #[inline(always)]
    pub fn g(range: Range, gs: f32) -> Self {
        Self::mg(range, gs * 1000.0)
    }

    #[inline(always)]
    pub fn mg(range: Range, mgs: f32) -> Self {
        let value = mgs / (range.as_mg() as f32);

        let result = crude_ceil(value);

        Threshold(result.try_into().unwrap())
    }

    pub const ZERO: Self = Threshold(0);
}

/// a crude `.ceil()`, the std one is not currently available when using no_std
fn crude_ceil(value: f32) -> u64 {
    let truncated = value as u64;

    let round_up = value - (truncated as f32) > 0.0;

    if round_up {
        truncated + 1
    } else {
        truncated
    }
}

/// Output data rate.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum DataRate {
    /// 400Hz (Default)
    Hz_400 = 0b0111,

    /// 200Hz
    Hz_200 = 0b0110,

    /// 100Hz
    Hz_100 = 0b0101,

    /// 50Hz
    Hz_50 = 0b0100,

    /// 25Hz
    Hz_25 = 0b0011,

    /// 10Hz
    Hz_10 = 0b0010,

    /// 1Hz
    Hz_1 = 0b0001,

    /// Power down
    PowerDown = 0b0000,
}

impl DataRate {
    pub const fn bits(self) -> u8 {
        self as u8
    }

    pub const fn sample_rate(self) -> f32 {
        match self {
            DataRate::Hz_400 => 400.0,
            DataRate::Hz_200 => 200.0,
            DataRate::Hz_100 => 100.0,
            DataRate::Hz_50 => 50.0,
            DataRate::Hz_25 => 25.0,
            DataRate::Hz_10 => 10.0,
            DataRate::Hz_1 => 1.0,
            DataRate::PowerDown => 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Duration(pub(crate) u8);

impl Duration {
    /// Convert a number of seconds into a duration. Internally a duration is represented
    /// as a multiple of `1 / ODR` where ODR (the output data rate) is of type [`DataRate`].
    #[inline(always)]
    pub fn seconds(output_data_rate: DataRate, seconds: f32) -> Self {
        let duration = output_data_rate.sample_rate() * seconds;

        Self(duration as u8)
    }

    /// Convert a number of miliseconds into a duration. Internally a duration is represented
    /// as a multiple of `1 / ODR` where ODR (the output data rate) is of type [`DataRate`].
    ///
    ///     assert_eq!(Duration::miliseconds(DataRate::Hz_400, 25.0), 10);
    #[inline(always)]
    pub fn miliseconds(output_data_rate: DataRate, miliseconds: f32) -> Self {
        Self::seconds(output_data_rate, miliseconds * 1000.0)
    }

    pub const ZERO: Self = Duration(0);
}

/// Data status structure. Decoded from the `STATUS_REG` register.
///
/// `STATUS_REG` has the following bit fields:
///   * `ZYXOR` - X, Y and Z-axis data overrun
///   * `ZOR` - Z-axis data overrun
///   * `YOR` - Y-axis data overrun
///   * `XOR` - X-axis data overrun
///   * `ZYXDA` - X, Y and Z-axis new data available
///   * `ZDA` - Z-axis new data available
///   * `YDA` Y-axis new data available
///   * `XDA` X-axis new data available
///
/// This struct splits the fields into more convenient groups:
///  * `zyxor` -> `ZYXOR`
///  * `xyzor` -> (`XOR`, `YOR`, `ZOR`)
///  * `zyxda` -> `ZYXDA`
///  * `xyzda` -> (`XDA`, `YDA`, `ZDA`)
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,

    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),

    /// ZYXDA bit
    pub zyxda: bool,

    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}

/// Information about what is stored in the FIFO
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FifoStatus {
    /// The watermark bit is set high when FIFO content exceeds watermark level
    pub watermark: bool,
    /// The overrun bit is set high when FIFO buffer is full; this means that the FIFO buffer
    /// contains 32 unread samples. At the following ODR a new sample set replaces the
    /// oldest FIFO value. The OVRN bit is set to 0 when the first sample set has been
    /// read
    pub overrun: bool,
    /// The empty bit is set high when all FIFO samples have been read and FIFO is empty
    pub empty: bool,
    /// The current number of unread samples stored in the
    /// FIFO buffer. When FIFO is enabled, this value increases
    /// at ODR frequency until the buffer is full, whereas,
    /// it decreases every time one sample set is retrieved from FIFO.
    pub stack_size: u8,
}

impl FifoStatus {
    /// Interpret the content of the `FIFO_SRC_REG` register
    pub const fn from_bits(status: u8) -> Self {
        Self {
            watermark: (status >> 7) & 1 == 1,
            overrun: (status >> 6) & 1 == 1,
            empty: (status >> 5) & 1 == 1,
            stack_size: status & 0b0001_1111,
        }
    }
}

/// FIFO behavior. See [the spec](https://www.st.com/resource/en/datasheet/lis3dh.pdf#page=22) for
/// full details.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FifoMode {
    /// The FIFO is not operational
    ByPass,
    /// In FIFO mode, the buffer continues filling data from the X, Y and Z accelerometer channels
    /// until it is full (a set of 32 samples stored). When the FIFO is full, it stops collecting data from
    /// the input channels and the FIFO content remains unchanged.
    Fifo,
    /// In Stream mode the FIFO continues filling data from the X, Y, and Z accelerometer channels
    /// until the buffer is full (a set of 32 samples stored) at which point the FIFO buffer index
    /// restarts from the beginning and older data is replaced by the current data. The oldest values
    /// continue to be overwritten until a read operation frees the FIFO slots
    Stream,
    /// In Stream-to-FIFO mode, data from the X, Y and Z accelerometer channels are collected in
    /// a combination of Stream mode and FIFO mode. The FIFO buffer starts operating in Stream
    /// mode and switches to FIFO mode when interrupt 1 occurs.
    StreamToFifoInt1,
    /// In Stream-to-FIFO mode, data from the X, Y and Z accelerometer channels are collected in
    /// a combination of Stream mode and FIFO mode. The FIFO buffer starts operating in Stream
    /// mode and switches to FIFO mode when interrupt 2 occurs.
    StreamToFifoInt2,
}

impl FifoMode {
    /// Convert the mode to bits that can be written to the `FIFO_CTRL_REG` register.
    pub const fn to_bits(self) -> u8 {
        let mut trigger = false;

        let mode = match self {
            FifoMode::ByPass => 0b00,
            FifoMode::Fifo => 0b01,
            FifoMode::Stream => 0b10,
            FifoMode::StreamToFifoInt1 => 0b11,
            FifoMode::StreamToFifoInt2 => {
                trigger = true;

                0b11
            }
        };

        mode << 6 | (trigger as u8) << 5
    }
}

/// Operating mode.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// High-resolution mode (12-bit data output)
    HighResolution,

    /// Normal mode (10-bit data output)
    Normal,

    /// Low-power mode (8-bit data output)
    LowPower,
}

// === WHO_AMI_I (0Fh) ===

/// `WHO_AM_I` device identification register
pub const DEVICE_ID: u8 = 0x33;

// === TEMP_CFG_REG (1Fh) ===

pub const ADC_EN: u8 = 0b1000_0000;
pub const TEMP_EN: u8 = 0b0100_0000;

// === CTRL_REG1 (20h) ===

pub const ODR_MASK: u8 = 0b1111_0000;
pub const LP_EN: u8 = 0b0000_1000;
pub const Z_EN: u8 = 0b0000_0100;
pub const Y_EN: u8 = 0b0000_0010;
pub const X_EN: u8 = 0b0000_0001;

// === CTRL_REG4 (23h) ===

pub const BDU: u8 = 0b1000_0000;
pub const FS_MASK: u8 = 0b0011_0000;
pub const HR: u8 = 0b0000_1000;

// === STATUS_REG (27h) ===

pub const ZYXOR: u8 = 0b1000_0000;
pub const ZOR: u8 = 0b0100_0000;
pub const YOR: u8 = 0b0010_0000;
pub const XOR: u8 = 0b0001_0000;
pub const ZYXDA: u8 = 0b0000_1000;
pub const ZDA: u8 = 0b0000_0100;
pub const YDA: u8 = 0b0000_0010;
pub const XDA: u8 = 0b0000_0001;

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn threshold_g_vs_mg() {
        assert_eq!(
            Threshold::g(Range::G2, 1.5),
            Threshold::mg(Range::G2, 1500.0)
        );
    }

    #[test]
    fn duration_seconds_vs_miliseconds() {
        assert_eq!(
            Duration::seconds(DataRate::Hz_400, 1.5),
            Duration::miliseconds(DataRate::Hz_400, 1500.0)
        );
    }
}
