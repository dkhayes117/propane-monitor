#![no_main]
#![no_std]

use defmt_rtt as _; // global logger
use nrf9160_hal as _;
// use nrf9160_hal::pac;
// memory layout
// use pac::interrupt;
use panic_probe as _;
// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

/// State Machine
#[derive(Debug)]
pub struct State<S> {
    pub state: S,
}

/// List of possible states for the state machine
#[derive(Debug)]
pub struct Initialize;

#[derive(Debug)]
pub struct Sleep;

#[derive(Debug)]
pub struct Ready;

#[derive(Debug)]
pub struct Sample;

#[derive(Debug)]
pub struct Transmit;

/// State constructor must begin in the Initialize state
impl State<Initialize> {
    pub fn new() -> State<Initialize> {
        State {
            state: Initialize {},
        }
    }
}

/// Initialize state can only transition to Ready state
impl State<Initialize> {
    pub fn next(self) -> State<Ready> {
        State { state: Ready {} }
    }
}

/// Sleep state can only transition to Ready state
impl State<Sleep> {
    pub fn next(self) -> State<Ready> {
        State { state: Ready {} }
    }
}

/// Ready state can only transition to Sample state
impl State<Ready> {
    pub fn next(self) -> State<Sample> {
        State { state: Sample {} }
    }
}

/// Sample state can transition to Sleep state or Transmit state based on if payload buffer is full
impl State<Sample> {
    pub fn payload_not_full_next(self) -> State<Sleep> {
        State { state: Sleep {} }
    }
    pub fn payload_full_next(self) -> State<Transmit> {
        State { state: Transmit {} }
    }
}

/// Transmit state can only transition to Sleep state
impl State<Transmit> {
    pub fn next(self) -> State<Sleep> {
        State { state: Sleep {} }
    }
}

// /// Interrupt Handler for LTE related hardware. Defer straight to the library.
// #[interrupt]
// #[allow(non_snake_case)]
// pub fn EGU1() {
//     nrfxlib::application_irq_handler();
//     cortex_m::asm::sev();
// }
//
// /// Interrupt Handler for LTE related hardware. Defer straight to the library.
// #[interrupt]
// #[allow(non_snake_case)]
// pub fn EGU2() {
//     nrfxlib::trace_irq_handler();
//     cortex_m::asm::sev();
// }
//
// /// Interrupt Handler for LTE related hardware. Defer straight to the library.
// #[interrupt]
// #[allow(non_snake_case)]
// pub fn IPC() {
//     nrfxlib::ipc_irq_handler();
//     cortex_m::asm::sev();
// }
// #[link_section = ".spm"]
// #[used]
// static SPM: [u8; 24052] = *include_bytes!("zephyr.bin");

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
