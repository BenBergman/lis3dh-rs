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

#[derive(Debug, Copy, Clone, Default)]
pub struct InterruptSource {
    pub and_or_combination: bool,
    pub interrupt_active: bool,

    pub z_axis_high: bool,
    pub z_axis_low: bool,

    pub y_axis_high: bool,
    pub y_axis_low: bool,

    pub x_axis_high: bool,
    pub x_axis_low: bool,
}

impl InterruptSource {
    pub const fn all() -> Self {
        Self {
            and_or_combination: true,
            interrupt_active: true,

            z_axis_high: true,
            z_axis_low: true,

            y_axis_high: true,
            y_axis_low: true,

            x_axis_high: true,
            x_axis_low: true,
        }
    }

    pub const fn high() -> Self {
        Self {
            and_or_combination: true,
            interrupt_active: true,

            z_axis_high: true,
            z_axis_low: false,

            y_axis_high: true,
            y_axis_low: false,

            x_axis_high: true,
            x_axis_low: false,
        }
    }

    pub const fn low() -> Self {
        Self {
            and_or_combination: true,
            interrupt_active: true,

            z_axis_high: false,
            z_axis_low: true,

            y_axis_high: false,
            y_axis_low: true,

            x_axis_high: false,
            x_axis_low: true,
        }
    }

    pub const fn bits(self) -> u8 {
        (self.and_or_combination as u8) << 7
            | (self.interrupt_active as u8) << 6
            | (self.z_axis_high as u8) << 5
            | (self.z_axis_low as u8) << 4
            | (self.y_axis_high as u8) << 3
            | (self.y_axis_low as u8) << 2
            | (self.x_axis_high as u8) << 1
            | (self.x_axis_low as u8) << 0
    }
}

#[derive(Debug, Copy, Clone, Default)]
pub struct IrqPin1Conf {
    pub click_en: bool,    // 7
    pub ia1_en: bool,      // 6
    pub ia2_en: bool,      // 5
    pub zyxda_en: bool,    // 4
    pub adc321da_en: bool, // 3
    pub wtm_en: bool,      // 2
    pub overrun_en: bool,  // 1
}

#[derive(Debug, Copy, Clone, Default)]
pub struct IrqPin2Conf {
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

impl IrqPin for IrqPin1Conf {
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

impl IrqPin for IrqPin2Conf {
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
