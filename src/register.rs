use num_enum::TryFromPrimitive;

/// Possible I²C slave addresses
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
        return self as u8;
    }
}

/// Enumerate all device registers
#[allow(non_camel_case_types)]
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
        match self {
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
            | Register::CLICK_SRC => true,
            _ => false,
        }
    }
}

/// Full-scale selection
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
    pub fn bits(self) -> u8 {
        self as u8
    }
}

/// Output data rate
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

    /// Low power mode (1.6KHz)
    LowPower_1K6HZ = 0b1000,

    /// # Normal power mode (1344Hz) / Low power mode (5KHz)
    Hz_1344_LP5k = 0b1001,
}

impl DataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

/// Data status structure. Decoded from `STATUS_REG` register.
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

/// Operating mode
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

// === CTRL_REG5 (24h) ===

pub const BOOT: u8 = 0b1000_0000;

// === STATUS_REG (27h) ===

pub const ZYXOR: u8 = 0b1000_0000;
pub const ZOR: u8 = 0b0100_0000;
pub const YOR: u8 = 0b0010_0000;
pub const XOR: u8 = 0b0001_0000;
pub const ZYXDA: u8 = 0b0000_1000;
pub const ZDA: u8 = 0b0000_0100;
pub const YDA: u8 = 0b0000_0010;
pub const XDA: u8 = 0b0000_0001;
