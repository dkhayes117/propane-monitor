#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

use defmt::{println, unwrap};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use heapless::Vec;
use nrf9160_hal::saadc::SaadcConfig;
use nrf9160_hal::{
    gpio,
    pac::{self, interrupt},
    prelude::*,
    pwm,
    pwm::Pwm,
    Saadc,
};
use nrf_modem_nal::embedded_nal::{heapless, SocketAddr, UdpClientStack};
use propane_monitor as _; // global logger + panicking-behavior + memory layout

// const MILLISECOND_CYCLES: u32 = nrf9160_hal::Timer::<pac::TIMER0_NS>::TICKS_PER_SECOND / 1000;

// A static buffer to hold data between data transfers which lives on the stack
// Mutex and RefCell are to ensure safe mutability
// Used for async programming
// static mut PAYLOAD_BUFFER: Mutex<RefCell<Vec<u8, 6>>> = Mutex::new(RefCell::new(Vec::new()));

#[cortex_m_rt::entry]
fn main() -> ! {
    // Take ownership of core and device peripherals
    let mut cp = unwrap!(cortex_m::Peripherals::take());
    let dp = unwrap!(pac::Peripherals::take());

    // A static buffer to hold data between data transfers which lives on the stack
    let mut payload_buffer: Vec<u8,6> = Vec::new();

    // Get handle for port0 GPIO
    let port0 = gpio::p0::Parts::new(dp.P0_NS);

    // Disable uarte0, uarte1, and p0_29 and leave off to reduce power consumption
    port0.p0_29.into_disconnected();
    dp.UARTE0_NS.enable.write(|w| w.enable().disabled());
    dp.UARTE1_NS.enable.write(|w| w.enable().disabled());

    // Configuration for reading hall effect voltage output
    let adc_config = SaadcConfig::default();
    let mut adc = Saadc::new(p.SAADC_NS, adc_config);

    // Icarus uses the P0.13 pin internally for battery voltage measurement
    // P0.14 works on both Icarus and Stratus boards, or replace with a usable analog pin on your board
    let mut adc_pin = port0.p0_14.into_floating_input();

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
    // let mut modem = nrf_modem_nal::Modem::new(None).unwrap();
    // let mut lte = modem.lte_socket().unwrap();
    // modem.lte_connect(&mut lte).unwrap();
    //
    // let mut udp_socket = modem.socket().unwrap();
    //
    // modem.connect(
    //     &mut udp_socket,
    //     SocketAddr::V4("142.250.179.211:80".parse().unwrap())
    // ).unwrap();

    let mut sum = 0 as usize;

    // Take 10 adc measurements and calculate mean
    for _ in 0..11 {
        sum += adc.read(&mut adc_pin).unwrap();
    }

    // Push tank level value to payload buffer, if the buffer is
    payload_buffer.push((sum / 10) as u8).unwrap();

    if payload_buffer.is_full() {
        println!("{payload.buffer}");
    }
    propane_monitor::exit();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
#[allow(non_snake_case)]
fn EGU1() {
    nrfxlib::application_irq_handler();
    cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
#[allow(non_snake_case)]
fn EGU2() {
    nrfxlib::trace_irq_handler();
    cortex_m::asm::sev();
}

/// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
#[allow(non_snake_case)]
fn IPC() {
    nrfxlib::ipc_irq_handler();
    cortex_m::asm::sev();
}
