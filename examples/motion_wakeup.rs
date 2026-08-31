#![no_std]
#![no_main]

use circuit_playground_express as bsp;
extern crate panic_halt;

use bsp::hal;
use bsp::pac::{CorePeripherals, Peripherals};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::sercom::i2c;

use lis3dh::{
    DataRate, Detect4D, Duration, HighPassFilterConfig, Interrupt1, InterruptConfig, InterruptMode,
    IrqPin1Config, LatchInterruptRequest, Lis3dh, Range, SlaveAddr, Threshold,
};

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
