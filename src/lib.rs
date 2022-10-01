#![no_main]
#![no_std]

use defmt::Format;
use defmt_rtt as _;
use heapless::Vec;
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
    // ModemError(nrfxlib::Error),
    LTEConnectionFailure,
    TransmitFailure,
    UndefinedTransition,
}

// impl From<nrfxlib::Error> for Error {
//     fn from(e: nrfxlib::Error) -> Self {
//         Self::ModemError(e)
//     }
// }

// Define our possible states
#[derive(Format, Clone, Copy)]
pub enum State {
    Initialize,
    Sleep,
    Ready,
    Sample,
    Transmit,
    Failure(Error),
}

// Define events
#[derive(Format)]
pub enum Event {
    SetupComplete,
    TimerInterrupt,
    SensorPowerOn,
    BufferFull,
    BufferNotFull,
    DataSent,
}

// Define transitions based on current state and event
impl State {
    pub fn step(self, event: Event) -> State {
        match (self, event) {
            (State::Sleep, Event::TimerInterrupt) => State::Ready,
            (State::Initialize, Event::SetupComplete) => State::Ready,
            (State::Ready, Event::SensorPowerOn) => State::Sample,
            (State::Sample, Event::BufferNotFull) => State::Sleep,
            (State::Sample, Event::BufferFull) => State::Transmit,
            (State::Transmit, Event::DataSent) => State::Sleep,
            (_s, _e) => State::Failure(Error::UndefinedTransition), // ( s, e ) => { State::Failure( String::from("Invalid (state,event) combination: ({:?}, {:?})") ) },
        }
    }
}

// Structure to hold our payload buffer (heapless Vec)
pub struct Payload {
    pub data: Vec<i16, 3>,
}

impl Payload {
    pub fn new() -> Self {
        Payload { data: Vec::new() }
    }

    pub fn transmit_data() {
        //TODO: Add CoAP code to send data
    }
}

// #[link_section = ".spm"]
// #[used]
// static SPM: [u8; 24052] = *include_bytes!("zephyr.bin");
