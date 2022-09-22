#![no_main]
#![no_std]

use defmt::{Format, println};
use defmt_rtt as _;
use heapless::String;
// global logger
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

// This is our state machine.
#[derive(Clone, Copy, PartialEq, Format)]
#[allow(dead_code)]
pub enum State {
    Initialize,
    Sleep,
    Ready,
    Sample,
    Transmit,
    Failure, //(String<128>)
}

#[derive(Format, Clone, Copy)]
pub enum Event {
    SetupComplete,
    TimerInterrupt,
    SensorPowerOn,
    BufferFull,
    BufferNotFull,
    DataSent,
}

impl State {
    pub fn step(self, event: Event) -> State {
        match (self, event) {
            ( State::Sleep, Event::TimerInterrupt ) => { State::Ready },
            ( State::Initialize, Event::SetupComplete ) => { State::Ready },
            ( State::Ready, Event::SensorPowerOn ) => { State::Sample },
            ( State::Sample, Event::BufferNotFull ) => { State::Sleep },
            ( State::Sample, Event::BufferFull ) => { State::Transmit },
            ( State::Transmit, Event::DataSent ) => { State::Sleep },
            ( _s, _e ) => { State::Failure }
            // ( s, e ) => { State::Failure( String::from("Invalid (state,event) combination: ({:?}, {:?})") ) },
        }
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
