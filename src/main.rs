#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

// use defmt::{println, unwrap};
use nrf9160_hal::pac::{self, interrupt};
use nrf_modem_nal::embedded_nal::{SocketAddr, UdpClientStack};
use propane_monitor as _; // global logger + panicking-behavior + memory layout

// const MILLISECOND_CYCLES: u32 = nrf9160_hal::Timer::<pac::TIMER0_NS>::TICKS_PER_SECOND / 1000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = nrf9160_hal::pac::Peripherals::take().unwrap();

    // Disable uarte to reduce power consumption
    dp.UARTE0_NS.enable.write(|w| w.enable().disabled());

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
    let mut modem = nrf_modem_nal::Modem::new(None).unwrap();
    let mut lte = modem.lte_socket().unwrap();
    modem.lte_connect(&mut lte).unwrap();

    let mut udp_socket = modem.socket().unwrap();

    modem.connect(
        &mut udp_socket,
        SocketAddr::V4("142.250.179.211:80".parse().unwrap())
    ).unwrap();


    propane_monitor::exit();
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
