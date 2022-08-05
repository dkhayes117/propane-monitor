#![no_main]
#![no_std]

use cortex_m_rt::entry;
use cortex_m::asm::delay;
use nrf9160_hal::{gpio, pac::Peripherals, prelude::*, pwm, pwm::Pwm};
use propane_monitor as _;
use crate::gpio::Level;
// global logger + panicking-behavior + memory layout

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let port0 = gpio::p0::Parts::new(p.P0_NS);


    // Configure pin for PWM
    let pwm = Pwm::new(p.PWM0_NS);
    pwm.set_output_pin(
        pwm::Channel::C0,
        port0.p0_31.into_push_pull_output(Level::Low).degrade()
    );

    // 5% Duty Cycle at 50Hz for servo Control
    pwm.set_prescaler(pwm::Prescaler::Div128);
    pwm.set_period( 50u32.hz() );
    pwm.enable();

    // Cycle through servo rotational range
    for div in 10..=23{
        pwm.set_duty_off_common( pwm.max_duty() / div );
        delay(20_000_000);
    }

    propane_monitor::exit();
}
