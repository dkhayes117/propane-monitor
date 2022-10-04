#![no_main]
#![no_std]

use defmt::Format;
use defmt_rtt as _;

use serde::Serialize;
use heapless::Vec;
// use heapless::String;
// global logger
use nrf9160_hal as _;
// use nrf9160_hal::pac;
// memory layout
// use pac::interrupt;
use alloc_cortex_m::CortexMHeap;
use core::{
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, Ordering},
};
use coap_lite::{CoapRequest, ContentFormat, RequestType};
use nrf_modem_nal::embedded_nal::{nb, SocketAddr, UdpClientStack, Dns};
use nrf_modem_nal::Modem;

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
#[derive(Serialize)]
pub struct Payload {
    pub data: Vec<i16,3>,
}


impl Payload {
    pub fn new() -> Self {
        Payload { data: Vec::new() }
    }

    pub fn transmit_data(&mut self, modem: &mut Modem) {
        //TODO: Finish CoAP implementation
        let mut request = CoapRequest::new();
        request.set_method(RequestType::Post);
        request.set_path("coap://sandbox.drogue.cloud/v1/foo");
        request.message.set_content_format(ContentFormat::ApplicationJSON);
        request.message.payload = serde_json::to_vec(&self).unwrap();

        // Connect LTE and UDP Socket for data transfer
        let mut lte = modem.lte_socket().unwrap();
        modem.lte_connect(&mut lte).unwrap();

        nb::block!(modem.lte_connect(&mut lte)).unwrap();

        let mut udp_socket = modem.socket().unwrap();

        // TODO: Change/check connect address
        nb::block!(modem.connect(
            &mut udp_socket,
            //SocketAddr::V4("142.250.179.211:80".parse().unwrap())
            modem.get_host_by_name(&request.get_path(),nrf_modem_nal::embedded_nal::AddrType::IPv4).unwrap(),
        )).unwrap(); // ip.jsontest.com

        nb::block!(modem.send(
            &mut udp_socket,
            &request.message.to_bytes().unwrap()
        )).unwrap();

        //clear buffer after upload
        self.data.clear();

        // Ok(())
    }
}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

static mut HEAP_DATA: [MaybeUninit<u8>; 8192] = [MaybeUninit::uninit(); 8192];

pub fn init() {
    static ONCE: AtomicBool = AtomicBool::new(false);

    if ONCE
        .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
        .is_ok()
    {
        unsafe {
            ALLOCATOR.init(HEAP_DATA.as_ptr() as usize, HEAP_DATA.len());
        }
    }
}
// #[link_section = ".spm"]
// #[used]
// static SPM: [u8; 24052] = *include_bytes!("zephyr.bin");
