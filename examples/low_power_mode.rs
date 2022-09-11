#![no_main]
#![no_std]

extern crate tinyrlibc;
use cortex_m::asm::wfe;
use nrf9160_hal::gpio;
// use defmt::println;
use nrf9160_hal::pac::{self, interrupt};
use propane_monitor as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let port0 = gpio::p0::Parts::new(dp.P0_NS);
    // Enable the modem interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EGU1);
        pac::NVIC::unmask(pac::Interrupt::EGU2);
        pac::NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::EGU1, 4 << 5);
        cp.NVIC.set_priority(pac::Interrupt::EGU2, 4 << 5);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    port0.p0_29.into_disconnected();

    // Initialize the modem
    nrfxlib::init().unwrap();
    nrfxlib::modem::set_system_mode(nrfxlib::modem::SystemMode::LteM).unwrap();
    nrfxlib::modem::on().unwrap();
    // nrfxlib::at::send_at_command(r#"AT+CPSMS=1,"","","00100001","00000000""#,|_| {}).unwrap();
    nrfxlib::modem::off().unwrap();

    // Disable uarte to reduce power consumption
    dp.UARTE0_NS.enable.write(|w| w.enable().disabled());
    dp.UARTE1_NS.enable.write(|w| w.enable().disabled());

    loop {
        wfe();
    }
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
