use crate::register::*;

#[derive(Debug, Copy, Clone)]
pub struct Interrupt1;

#[derive(Debug, Copy, Clone)]
pub struct Interrupt2;

pub trait Interrupt {
    fn cfg_reg() -> Register;
    fn ths_reg() -> Register;
    fn src_reg() -> Register;
    fn duration_reg() -> Register;
    fn lir_int_bit() -> u8;
    fn d4d_int_bit() -> u8;
}

impl Interrupt for Interrupt1 {
    fn cfg_reg() -> Register {
        Register::INT1_CFG
    }

    fn ths_reg() -> Register {
        Register::INT1_THS
    }

    fn src_reg() -> Register {
        Register::INT1_SRC
    }

    fn duration_reg() -> Register {
        Register::INT1_DURATION
    }

    fn lir_int_bit() -> u8 {
        3
    }

    fn d4d_int_bit() -> u8 {
        2
    }
}

impl Interrupt for Interrupt2 {
    fn cfg_reg() -> Register {
        Register::INT2_CFG
    }

    fn ths_reg() -> Register {
        Register::INT2_THS
    }

    fn src_reg() -> Register {
        Register::INT2_SRC
    }

    fn duration_reg() -> Register {
        Register::INT2_DURATION
    }

    fn lir_int_bit() -> u8 {
        1
    }

    fn d4d_int_bit() -> u8 {
        0
    }
}

/// When to generate an interrupt.
///
/// Internal representation:
///
/// | AOI | 6D | Interrupt mode |
/// | - | - | --- |
/// | 0 | 0 | OR combination of interrupt events  |
/// | 0 | 1 | 6-direction movement recognition  |
/// | 1 | 0 | AND combination of interrupt events  |
/// | 1 | 1 | 6-direction position recognition  |
#[derive(Debug, Copy, Clone)]
pub enum InterruptMode {
    OrCombination = 0b00 << 6,
    Movement = 0b01 << 6,
    AndCombination = 0b10 << 6,
    Position = 0b11 << 6,
}

impl Default for InterruptMode {
    fn default() -> Self {
        InterruptMode::OrCombination
    }
}

impl InterruptMode {
    const fn from_bits(input: u8) -> Self {
        match input >> 6 {
            0b00 => InterruptMode::OrCombination,
            0b01 => InterruptMode::Movement,
            0b10 => InterruptMode::AndCombination,
            0b11 => InterruptMode::Position,
            _ => {
                // change to unreachable when https://github.com/rust-lang/rust/issues/51999 is stable
                InterruptMode::Position
            }
        }
    }
    const fn to_bits(self) -> u8 {
        match self {
            InterruptMode::OrCombination => 0b00 << 6,
            InterruptMode::Movement => 0b01 << 6,
            InterruptMode::AndCombination => 0b10 << 6,
            InterruptMode::Position => 0b11 << 6,
        }
    }
}

impl From<u8> for InterruptMode {
    fn from(input: u8) -> Self {
        Self::from_bits(input)
    }
}

/// Configure which events on which axes trigger an interrupt.
#[derive(Debug, Copy, Clone, Default)]
#[doc(alias = "INT1_CFG")]
#[doc(alias = "INT2_CFG")]
pub struct InterruptConfig {
    pub z_axis_high: bool,
    pub z_axis_low: bool,

    pub y_axis_high: bool,
    pub y_axis_low: bool,

    pub x_axis_high: bool,
    pub x_axis_low: bool,
}

impl InterruptConfig {
    /// Don't generate an interrupt for any event
    pub const fn none() -> Self {
        Self {
            z_axis_high: false,
            z_axis_low: false,

            y_axis_high: false,
            y_axis_low: false,

            x_axis_high: false,
            x_axis_low: false,
        }
    }

    /// Generate an interrupt for a low and a high event
    /// on any of the axes
    pub const fn high_and_low() -> Self {
        Self {
            z_axis_high: true,
            z_axis_low: true,

            y_axis_high: true,
            y_axis_low: true,

            x_axis_high: true,
            x_axis_low: true,
        }
    }

    /// Generate an interrupt for a high event
    /// on any of the axes
    pub const fn high() -> Self {
        Self {
            z_axis_high: true,
            z_axis_low: false,

            y_axis_high: true,
            y_axis_low: false,

            x_axis_high: true,
            x_axis_low: false,
        }
    }

    /// Generate an interrupt for a low event
    /// on any of the axes
    pub const fn low() -> Self {
        Self {
            z_axis_high: false,
            z_axis_low: true,

            y_axis_high: false,
            y_axis_low: true,

            x_axis_high: false,
            x_axis_low: true,
        }
    }

    pub fn to_bits(self, interrupt_mode: InterruptMode) -> u8 {
        interrupt_mode.to_bits()
            | (self.z_axis_high as u8) << 5
            | (self.z_axis_low as u8) << 4
            | (self.y_axis_high as u8) << 3
            | (self.y_axis_low as u8) << 2
            | (self.x_axis_high as u8) << 1
            | (self.x_axis_low as u8)
    }

    pub const fn from_bits(irq_src: u8) -> Self {
        Self {
            z_axis_high: irq_src & (1 << 5) != 0,
            z_axis_low: irq_src & (1 << 4) != 0,
            y_axis_high: irq_src & (1 << 3) != 0,
            y_axis_low: irq_src & (1 << 2) != 0,
            x_axis_high: irq_src & (1 << 1) != 0,
            x_axis_low: irq_src & (1 << 0) != 0,
        }
    }
}

#[derive(Debug, Copy, Clone, Default)]
#[doc(alias = "CTRL_REG3")]
pub struct IrqPin1Config {
    pub click_en: bool,    // 7
    pub ia1_en: bool,      // 6
    pub ia2_en: bool,      // 5
    pub zyxda_en: bool,    // 4
    pub adc321da_en: bool, // 3
    pub wtm_en: bool,      // 2
    pub overrun_en: bool,  // 1
}

#[derive(Debug, Copy, Clone, Default)]
#[doc(alias = "CTRL_REG6")]
pub struct IrqPin2Config {
    pub click_en: bool,   // 7
    pub ia1_en: bool,     // 6
    pub ia2_en: bool,     // 5
    pub boot_en: bool,    // 4
    pub act_en: bool,     // 3
    pub active_low: bool, // 1
}

pub trait IrqPin {
    fn ctrl_reg() -> Register;
    fn bits(self) -> u8;
}

impl IrqPin for IrqPin1Config {
    fn ctrl_reg() -> Register {
        Register::CTRL3
    }

    fn bits(self) -> u8 {
        (self.click_en as u8) << 7
            | (self.ia1_en as u8) << 6
            | (self.ia2_en as u8) << 5
            | (self.zyxda_en as u8) << 4
            | (self.adc321da_en as u8) << 3
            | (self.wtm_en as u8) << 2
            | (self.overrun_en as u8) << 1
    }
}

impl IrqPin for IrqPin2Config {
    fn ctrl_reg() -> Register {
        Register::CTRL6
    }

    fn bits(self) -> u8 {
        (self.click_en as u8) << 7
            | (self.ia1_en as u8) << 6
            | (self.ia2_en as u8) << 5
            | (self.boot_en as u8) << 4
            | (self.act_en as u8) << 3
            | (self.active_low as u8) << 1
    }
}

#[derive(Debug, Copy, Clone, Default)]
#[doc(alias = "INT1_SRC")]
#[doc(alias = "INT2_SRC")]
pub struct InterruptSource {
    pub interrupt_active: bool,

    pub z_axis_high: bool,
    pub z_axis_low: bool,

    pub y_axis_high: bool,
    pub y_axis_low: bool,

    pub x_axis_high: bool,
    pub x_axis_low: bool,
}

impl InterruptSource {
    pub const fn from_bits(input: u8) -> Self {
        // NOTE the leftmost bit is unused
        Self {
            interrupt_active: input & (1 << 6) != 0,
            z_axis_high: input & (1 << 5) != 0,
            z_axis_low: input & (1 << 4) != 0,
            y_axis_high: input & (1 << 3) != 0,
            y_axis_low: input & (1 << 2) != 0,
            x_axis_high: input & (1 << 1) != 0,
            x_axis_low: input & (1 << 0) != 0,
        }
    }
}

/// Latch (keep active) the interrupt until the [`get_irq_src`] is read.
///
/// [`get_irq_src`]: crate::Lis3dh::get_irq_src
#[derive(Debug, Copy, Clone)]
pub enum LatchInterruptRequest {
    Enable,
    Disable,
}

impl Default for LatchInterruptRequest {
    fn default() -> Self {
        LatchInterruptRequest::Disable
    }
}

impl From<bool> for LatchInterruptRequest {
    fn from(input: bool) -> Self {
        if input {
            Self::Enable
        } else {
            Self::Disable
        }
    }
}

/// 4D detection is a subset of the 6D detection where detection on the Z axis is disabled.
/// This setting only has effect when the interrupt mode is either `Movement` or `Position`.
#[derive(Debug, Copy, Clone)]
pub enum Detect4D {
    Enable,
    Disable,
}

impl Default for Detect4D {
    fn default() -> Self {
        Detect4D::Disable
    }
}

impl From<bool> for Detect4D {
    fn from(input: bool) -> Self {
        if input {
            Self::Enable
        } else {
            Self::Disable
        }
    }
}
