#![no_std]
#![no_main]

use circuit_playground_express as hal;
extern crate panic_halt;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::{CorePeripherals, Peripherals};
use hal::sercom::{PadPin, I2CMaster1};
use hal::time::KiloHertz;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use lis3dh::Lis3dh;

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

    let mut lis3dh = Lis3dh::new(i2c, 0x19).unwrap();
    lis3dh.set_range(lis3dh::Range::G8).unwrap();
    let mut delay = Delay::new(core.SYST, &mut clocks);
    
    let mut tracker = lis3dh.try_into_tracker().unwrap();

    loop {
        let orientation = tracker.orientation().unwrap();
        hprintln!("{:?}", orientation).ok();
        delay.delay_ms(1000u16)
    }
}
