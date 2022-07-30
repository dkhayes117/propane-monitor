#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

use defmt::unwrap;
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
        "AT+CFUN=1",      // Sets Radio to Normal
        "AT+CFUN?",       // Read Radio Status
        "AT+CEREG=5",     // Sets current network registration status result codes to level 5
        "AT+CEREG?",      // Current network registration status
        "AT%XSNRSQ?",     // Signal Noise Ratio
        "AT+CESQ",        // Signal Quality
        "AT%XTEMP?",      // Internal Temperature
        "AT+CGCONTRDP=0", //
        "AT+CGCONTRDP?",
        "AT+CCLK?",      // Reads Real-Time Clock
        "AT%XMONITOR",   // Read modem parameters
        "AT+CGDCONT?",   // List of defined contexts
        "AT+CGPADDR",    // Test command returns a list of <cid> values
        "AT%XCONNSTAT?", // Reads connectivity statistics
    ] {
        print_at_results(cmd);
    }

    // nrfxlib::at::send_at_command(cmd, |r| defmt::println!("Response: {}", r));

    propane_monitor::exit();
}

/// Print AT command results
fn print_at_results(cmd: &str) {
    if let Err(_e) = nrfxlib::at::send_at_command(cmd, |s| {
        defmt::println!("> {}", s);
    }) {
        defmt::println!("Err running {}: error", cmd);
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
