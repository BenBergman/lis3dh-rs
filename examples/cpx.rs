#![no_std]
#![no_main]

use circuit_playground_express as hal;
extern crate panic_halt;

use accelerometer::{RawAccelerometer, Tracker};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::sercom::{I2CMaster1, PadPin};
use hal::time::KiloHertz;

use lis3dh::{Lis3dh, SlaveAddr};

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut pins = hal::Pins::new(peripherals.PORT);
    let gclk0 = clocks.gclk0();

    let i2c = I2CMaster1::new(
        &clocks.sercom1_core(&gclk0).unwrap(),
        KiloHertz(400),
        peripherals.SERCOM1,
        &mut peripherals.PM,
        pins.accel_sda.into_pad(&mut pins.port),
        pins.accel_scl.into_pad(&mut pins.port),
    );

    let mut lis3dh = Lis3dh::new(i2c, SlaveAddr::Alternate).unwrap();
    lis3dh.set_range(lis3dh::Range::G8).unwrap();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut tracker = Tracker::new(3700.0);

    loop {
        let accel = lis3dh.accel_raw().unwrap();
        let orientation = tracker.update(accel);
        hprintln!("{:?}", orientation).ok();
        delay.delay_ms(1000u16)
    }
}
