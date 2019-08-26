#![allow(non_camel_case_types)]

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    STATUS_AUX      = 0x07,
    OUT_ADC1_L      = 0x08,
    OUT_ADC1_H      = 0x09,
    OUT_ADC2_L      = 0x0A,
    OUT_ADC2_H      = 0x0B,
    OUT_ADC3_L      = 0x0C,
    OUT_ADC3_H      = 0x0D,
    WHOAMI          = 0x0F,
    CTRL0           = 0x1E,
    TEMP_CFG        = 0x1F,
    CTRL1           = 0x20,
    CTRL2           = 0x21,
    CTRL3           = 0x22,
    CTRL4           = 0x23,
    CTRL5           = 0x24,
    CTRL6           = 0x25,
    REFERENCE       = 0x26,
    STATUS          = 0x27,
    OUT_X_L         = 0x28,
    OUT_X_H         = 0x29,
    OUT_Y_L         = 0x2A,
    OUT_Y_H         = 0x2B,
    OUT_Z_L         = 0x2C,
    OUT_Z_H         = 0x2D,
    FIFO_CTRL       = 0x2E,
    FIFO_SRC        = 0x2F,
    INT1_CFG        = 0x30,
    INT1_SRC        = 0x31,
    INT1_THS        = 0x32,
    INT1_DURATION   = 0x33,
    INT2_CFG        = 0x34,
    INT2_SRC        = 0x35,
    INT2_THS        = 0x36,
    INT2_DURATION   = 0x37,
    CLICK_CFG       = 0x38,
    CLICK_SRC       = 0x39,
    CLICK_THS       = 0x3A,
    TIME_LIMIT      = 0x3B,
    TIME_LATENCY    = 0x3C,
    TIME_WINDOW     = 0x3D,
    ACT_THS         = 0x3E,
    ACT_DUR         = 0x3F,
}

impl Register {
    pub fn addr(self) -> u8 {
        self as u8
    }

    pub fn read_only(self) -> bool {
        match self {
            Register::STATUS_AUX |
            Register::OUT_ADC1_L |
            Register::OUT_ADC1_H |
            Register::OUT_ADC2_L |
            Register::OUT_ADC2_H |
            Register::OUT_ADC3_L |
            Register::OUT_ADC3_H |
            Register::WHOAMI |
            Register::STATUS |
            Register::OUT_X_L |
            Register::OUT_X_H |
            Register::OUT_Y_L |
            Register::OUT_Y_H |
            Register::OUT_Z_L |
            Register::OUT_Z_H |
            Register::FIFO_SRC |
            Register::INT1_SRC |
            Register::INT2_SRC |
            Register::CLICK_SRC  => true,
            _ => false,
        }
    }
}

