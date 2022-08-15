//! A simple program that connects to serial and relays the socket AT commands to it.
//! With this you can talk directly to the nrf modem with nrfConnect LTE Link Monitor

#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

use nrf9160_hal::{
    gpio,
    pac::{self, interrupt},
    uarte,
};
use propane_monitor as _; // global logger + panicking-behavior + memory layout

const MILLISECOND_CYCLES: u32 = nrf9160_hal::Timer::<pac::TIMER0_NS>::TICKS_PER_SECOND / 1000;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Take ownership of the chip and device peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = nrf9160_hal::pac::Peripherals::take().unwrap();

    // Enable the modem interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EGU1);
        pac::NVIC::unmask(pac::Interrupt::EGU2);
        pac::NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::EGU1, 4 << 5);
        cp.NVIC.set_priority(pac::Interrupt::EGU2, 4 << 5);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    // Set up the serial: Icarus Pins(6,9), Stratus Pins(5,6)
    let pins0 = gpio::p0::Parts::new(dp.P0_NS);
    let uart_pins = uarte::Pins {
        rxd: pins0.p0_05.into_floating_input().degrade(),
        txd: pins0
            .p0_06
            .into_push_pull_output(gpio::Level::High)
            .degrade(),
        cts: None,
        rts: None,
    };
    let mut serial = nrf9160_hal::Uarte::new(
        dp.UARTE0_NS,
        uart_pins,
        uarte::Parity::EXCLUDED,
        uarte::Baudrate::BAUD115200,
    );
    let mut serial_timer = nrf9160_hal::Timer::new(dp.TIMER0_NS);

    // Initialize the modem
    nrfxlib::init().unwrap();
    let at_socket = nrfxlib::at::AtSocket::new().unwrap();

    let mut buffer = [0; 1024];

    loop {
        // Write the response from the AT Socket to the serial console
        // The nrfxlib is written in C which has null terminated strings
        // write length - 1 to not send nulls to LTE Link
        if let Some(length) = at_socket.recv(&mut buffer).unwrap() {
            if length != 0 {
                serial.write(&buffer[..length-1]).unwrap();
            }
        }
        // Read blocking the AT command input into our buffer with a 20ms timeout
        // Then send the command to the at_socket and wait for the response
        // 20 ms worked the best with LTE Link, this can be increased for manual typing
        if let Err(nrf9160_hal::uarte::Error::Timeout(length)) =
            serial.read_timeout(&mut buffer, &mut serial_timer, MILLISECOND_CYCLES * 20)
        {
            if length != 0 {
                at_socket.write(&buffer[..length]).unwrap();
            }
        }
    }
}

// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn EGU1() {
    nrfxlib::application_irq_handler();
    cortex_m::asm::sev();
}

// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn EGU2() {
    nrfxlib::trace_irq_handler();
    cortex_m::asm::sev();
}

// Interrupt Handler for LTE related hardware. Defer straight to the library.
#[interrupt]
fn IPC() {
    nrfxlib::ipc_irq_handler();
    cortex_m::asm::sev();
}
