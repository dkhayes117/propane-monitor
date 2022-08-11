#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

use defmt::{println, unwrap};
use nrf9160_hal::pac::{self, interrupt};
use propane_monitor as _; // global logger + panicking-behavior + memory layout

// const MILLISECOND_CYCLES: u32 = nrf9160_hal::Timer::<pac::TIMER0_NS>::TICKS_PER_SECOND / 1000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut cp = unwrap!(cortex_m::Peripherals::take());
    let _dp = unwrap!(nrf9160_hal::pac::Peripherals::take());

    // Enable the modem interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EGU1);
        pac::NVIC::unmask(pac::Interrupt::EGU2);
        pac::NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::EGU1, 4 << 5);
        cp.NVIC.set_priority(pac::Interrupt::EGU2, 4 << 5);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    // Initialize the modem
    nrfxlib::init().unwrap();

    for cmd in [
        "AT%SHORTSWVER",         // Check modem firmware version
        "AT%XBANDLOCK=1,'1000'", // Set permanent band-lock on Band 4
        "AT+CFUN=1",             // Sets Radio to Normal
        "AT+CFUN?",              // Read Radio Status
        "AT%XCBAND",             // Check current band
        "AT+CFUN=0",             // Turn Radio Off
    ] {
        print_at_results(cmd);
    }

    propane_monitor::exit();
}

/// Print AT command results
fn print_at_results(cmd: &str) {
    if let Err(_e) = nrfxlib::at::send_at_command(cmd, |s| {
        println!("> {}", s);
    }) {
        println!("Err running {}: error", cmd);
    }
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn EGU1() {
    nrfxlib::application_irq_handler();
    cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn EGU2() {
    nrfxlib::trace_irq_handler();
    cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn IPC() {
    nrfxlib::ipc_irq_handler();
    cortex_m::asm::sev();
}
