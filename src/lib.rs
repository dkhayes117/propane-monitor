#![no_main]
#![no_std]

use defmt::{Format, println};
use defmt_rtt as _;
// use heapless::String;
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
// Error types
#[derive(Format, Clone, Copy)]
pub enum Error {
    LTEConnectionFailure,
    TransmitFailure,
    UndefinedTransition,
}

// This is our state machine.
#[derive(Format, Clone, Copy)]
pub enum State {
    Initialize,
    Sleep,
    Ready,
    Sample,
    Transmit,
    Failure(Error),
}

#[derive(Format)]
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
            ( _s, _e ) => { State::Failure( Error::UndefinedTransition ) }
            // ( s, e ) => { State::Failure( String::from("Invalid (state,event) combination: ({:?}, {:?})") ) },
        }
    }
}

// #[link_section = ".spm"]
// #[used]
// static SPM: [u8; 24052] = *include_bytes!("zephyr.bin");

