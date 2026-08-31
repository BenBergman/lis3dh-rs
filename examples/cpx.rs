#![no_std]
#![no_main]

use circuit_playground_express as bsp;
extern crate panic_halt;

use accelerometer::{RawAccelerometer, Tracker};
use bsp::hal;
use bsp::pac::{CorePeripherals, Peripherals};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::sercom::i2c;

use lis3dh::{Lis3dh, SlaveAddr};

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.pm,
        &mut peripherals.sysctrl,
        &mut peripherals.nvmctrl,
    );
    let pins = bsp::Pins::new(peripherals.port);
    let gclk0 = clocks.gclk0();

    let clock = clocks.sercom1_core(&gclk0).unwrap();
    let pads = i2c::Pads::new(pins.accel_sda, pins.accel_scl);
    let i2c = i2c::Config::new(&mut peripherals.pm, peripherals.sercom1, pads, clock.freq())
        .baud(400.kHz())
        .enable();

    let mut lis3dh = Lis3dh::new_i2c(i2c, SlaveAddr::Alternate).unwrap();
    lis3dh.set_range(lis3dh::Range::G8).unwrap();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut tracker = Tracker::new(3700.0_f32);

    loop {
        let accel = lis3dh.accel_raw().unwrap();
        let orientation = tracker.update(accel);
        hprintln!("{:?}", orientation).ok();
        delay.delay_ms(1000u16)
    }
}
