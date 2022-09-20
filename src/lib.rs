#![no_main]
#![no_std]

use defmt_rtt as _; // global logger
use defmt::Format;
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
#[derive(Clone,Copy,Format)]
#[allow(dead_code)]
pub struct State<S> {
    state: S,
}

/// List of possible states for the state machine
#[derive(Clone,Copy,Format)]
pub struct Initialize;

#[derive(Clone,Copy,Format)]
pub struct Sleep;

#[derive(Clone,Copy,Format)]
pub struct Ready;

#[derive(Clone,Copy,Format)]
pub struct Sample;

#[derive(Clone,Copy,Format)]
pub struct Transmit;

// Our Machine starts in the 'Waiting' state.
impl State<Initialize> {
    pub fn new() -> Self {
        State {
            state: Initialize {},
        }
    }
}

// The following are the defined transitions between states.
impl From<State<Initialize>> for State<Ready> {
    fn from(_val: State<Initialize>) -> State<Ready> {
        State {
            state: Ready {},
        }
    }
}

impl From<State<Sleep>> for State<Ready> {
    fn from(_val: State<Sleep>) -> State<Ready> {
        State {
            state: Ready {},
        }
    }
}

impl From<State<Ready>> for State<Sample> {
    fn from(_val: State<Ready>) -> State<Sample> {
        State {
            state: Sample {},
        }
    }
}

impl From<State<Sample>> for State<Sleep> {
    fn from(_val: State<Sample>) -> State<Sleep> {
        State {
            state: Sleep {},
        }
    }
}

impl From<State<Sample>> for State<Transmit> {
    fn from(_val: State<Sample>) -> State<Transmit> {
        State {
            state: Transmit {},
        }
    }
}

impl From<State<Transmit>> for State<Sleep> {
    fn from(_val: State<Transmit>) -> State<Sleep> {
        State {
            state: Sleep {},
        }
    }
}


// Here is we're building an enum so we can contain this state machine in a parent.
#[derive(Clone,Copy,Format)]
#[allow(dead_code)]
pub enum StateWrapper {
    Initialize(State<Initialize>),
    Sleep     (State<Sleep>),
    Ready     (State<Ready>),
    Sample    (State<Sample>),
    Transmit  (State<Transmit>),
}

// Defining a function which shifts the state along.
impl StateWrapper {
    pub fn step(mut self) -> Self {
        self = match self {
            StateWrapper::Initialize(val) => StateWrapper::Ready(val.into()),
            StateWrapper::Sleep(val) => StateWrapper::Ready(val.into()),
            StateWrapper::Ready(val) => StateWrapper::Sample(val.into()),
            StateWrapper::Sample(val) => StateWrapper::Transmit(val.into()),
            StateWrapper::Transmit(val) => StateWrapper::Sleep(val.into()),
        };
        self
    }
}

// The structure with a parent.
#[derive(Clone,Copy,Format)]
pub struct StateMachine {
    //bottle_filling_machine: BottleFillingMachineWrapper,
    pub state: StateWrapper,
}

impl StateMachine {
    pub fn new() -> Self {
        StateMachine {
            state: StateWrapper::Initialize(State::new()),
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
