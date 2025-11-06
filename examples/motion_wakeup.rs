#![no_std]
#![no_main]

use circuit_playground_express as hal;
extern crate panic_halt;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::sercom::{I2CMaster1, PadPin};
use hal::time::KiloHertz;

use lis3dh::{
    DataRate, Detect4D, Duration, HighPassFilterConfig, Interrupt1, InterruptConfig, InterruptMode,
    IrqPin1Config, LatchInterruptRequest, Lis3dh, Range, SlaveAddr, Threshold,
};

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

    let mut lis3dh = Lis3dh::new_i2c(i2c, SlaveAddr::Alternate).unwrap();

    let data_rate = DataRate::Hz_200;
    let wakeup_threshold = 500.0;
    let threshold = Threshold::mg(Range::G2, wakeup_threshold);

    // At 200Hz, each sample is 5ms. Duration of 0.020s = 4 samples
    let duration = Duration::seconds(data_rate, 0.020);

    // Set the output data rate
    lis3dh.set_datarate(data_rate).unwrap();

    // Set minimum acceleration threshold to trigger interrupt
    lis3dh
        .configure_irq_threshold(Interrupt1, threshold)
        .unwrap();

    // Set minimum duration threshold must be exceeded to trigger interrupt
    lis3dh.configure_irq_duration(Interrupt1, duration).unwrap();

    // Configure interrupt to trigger on high events (motion above threshold)
    // OR any axis, latched until read, 4D detection (ignore Z-axis rotation)
    lis3dh
        .configure_irq_src_and_control(
            Interrupt1,
            InterruptMode::OrCombination,
            InterruptConfig::high(),
            LatchInterruptRequest::Enable,
            Detect4D::Enable,
        )
        .unwrap();

    // Route interrupt 1 signal to physical INT1 pin
    lis3dh
        .configure_interrupt_pin(IrqPin1Config {
            ia1_en: true,
            ..Default::default()
        })
        .unwrap();

    // Enable high-pass filter for interrupt path to remove DC component (gravity)
    // This allows detection of motion/acceleration changes while ignoring gravity
    // Data output remains unfiltered so you still get absolute acceleration values
    lis3dh
        .configure_high_pass_filter(HighPassFilterConfig {
            enable_for_interrupt1: true,
            ..Default::default()
        })
        .unwrap();

    // Clear any stale latched interrupt before use
    let _ = lis3dh.get_irq_src(Interrupt1);

    let mut delay = Delay::new(core.SYST, &mut clocks);

    hprintln!("Motion wakeup configured. Waiting for movement...").ok();

    loop {
        // Check if interrupt was triggered
        let irq_src = lis3dh.get_irq_src(Interrupt1).unwrap();

        if irq_src.interrupt_active {
            hprintln!("Motion detected!").ok();
            hprintln!(
                "  X: {}, Y: {}, Z: {}",
                irq_src.x_axis_high,
                irq_src.y_axis_high,
                irq_src.z_axis_high
            )
            .ok();
        }

        delay.delay_ms(100u16);
    }
}
