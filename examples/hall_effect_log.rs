#![no_main]
#![no_std]

use crate::gpio::Level;
use cortex_m::asm::delay;
use cortex_m_rt::entry;
use nrf9160_hal::saadc::SaadcConfig;
use nrf9160_hal::{gpio, pac::Peripherals, prelude::*, pwm, pwm::Pwm, Saadc};
use propane_monitor as _;
// global logger + panicking-behavior + memory layout

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let port0 = gpio::p0::Parts::new(p.P0_NS);

    // Configuration for reading hall effect voltage output
    let adc_config = SaadcConfig::default();
    let mut adc = Saadc::new(p.SAADC_NS, adc_config);

    // Icarus uses the P0.13 pin internally for battery voltage measurement
    // P0.14 works on both Icarus and Stratus boards, or replace with a usable analog pin on your board
    let mut adc_pin = port0.p0_14.into_floating_input();

    // Configure pin for PWM
    let pwm = Pwm::new(p.PWM0_NS);
    pwm.set_output_pin(
        pwm::Channel::C0,
        port0.p0_31.into_push_pull_output(Level::Low).degrade(),
    );

    // 5% Duty Cycle at 50Hz for servo Control
    pwm.set_prescaler(pwm::Prescaler::Div128);
    pwm.set_period(50u32.hz());
    pwm.enable();

    // Array of tuples holding a calibrated duty_cycle for each gauge level
    let positions: [(u16, u16); 13] = [
        (5, 111),
        (10, 122),
        (15, 134),
        (20, 144),
        (25, 154),
        (30, 162),
        (40, 176),
        (50, 189),
        (60, 203),
        (70, 217),
        (80, 234),
        (85, 244),
        (88, 250),
    ];

    // Cycle through servo rotational range
    for (level, duty) in positions.iter() {
        // defmt::println!("Duty: {}",duty);
        pwm.set_duty_off_common(*duty);
        delay(300_000_000);
        defmt::println!(
            "Gauge Level: {}%, Voltage Level: {}",
            level,
            adc.read(&mut adc_pin).unwrap()
        );
    }

    delay(200_000_000);

    for div in 10..=23 {
        pwm.set_duty_off_common(pwm.max_duty() / div as u16);
        delay(1_000_000);
    }

    propane_monitor::exit();
}
