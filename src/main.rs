#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

use core::cell::RefCell;

use cortex_m::asm::wfe;
use cortex_m::interrupt::Mutex;

use defmt::{println, unwrap};
use heapless::Vec;

use nrf9160_hal::saadc::SaadcConfig;
use nrf9160_hal::{clocks, gpio, pac::{self, interrupt}, prelude::*, pwm, pwm::Pwm, rtc, Saadc};
use nrf9160_hal::pac::RTC0_NS;

use nrf_modem_nal::embedded_nal::{heapless, SocketAddr, UdpClientStack};
use propane_monitor as _; // global logger + panicking-behavior + memory layout
use propane_monitor::{StateMachine, StateWrapper};

// const MILLISECOND_CYCLES: u32 = nrf9160_hal::Timer::<pac::TIMER0_NS>::TICKS_PER_SECOND / 1000;

// How long to sleep: 15 seconds
static SLEEP_MS: u32 =  15_000;

// A static buffer to hold data between data transfers which lives on the stack
// Mutex and RefCell are to ensure safe mutability
// static mut PAYLOAD_BUFFER: Mutex<RefCell<Vec<u8, 6>>> = Mutex::new(RefCell::new(Vec::new()));

// Thread safe timer
static RTC: Mutex<RefCell<Option<rtc::Rtc<RTC0_NS>>>> = Mutex::new(RefCell::new(None));
// Global state
static DEVICE: Mutex<RefCell<Option<StateMachine>>> = Mutex::new(RefCell::new(None));


#[cortex_m_rt::entry]
fn main() -> ! {
    // Construct our StateMachine and place in the DEVICE static variable
    cortex_m::interrupt::free(|cs| {
        DEVICE.borrow(cs).replace(Some(StateMachine::new()));
    });

    // Take ownership of core and device peripherals
    let mut cp = unwrap!(cortex_m::Peripherals::take());
    let dp = unwrap!(pac::Peripherals::take());

    // A static buffer to hold data between data transfers which lives on the stack
    let mut payload_buffer: Vec<u8, 6> = Vec::new();

    // Get handle for port0 GPIO
    let port0 = gpio::p0::Parts::new(dp.P0_NS);

    // Disable uarte0, uarte1, and p0_29 and leave off to reduce power consumption
    port0.p0_29.into_disconnected();
    dp.UARTE0_NS.enable.write(|w| w.enable().disabled());
    dp.UARTE1_NS.enable.write(|w| w.enable().disabled());

    // Configuration for reading hall effect voltage output
    let adc_config = SaadcConfig::default();
    let mut adc = Saadc::new(dp.SAADC_NS, adc_config);

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

    // Enable the low-frequency-clock which is required by the RTC
    clocks::Clocks::new(dp.CLOCK_NS).start_lfclk();

    // Setup our timer so we can wake up to do our work periodically
    let prescaler = 0xFFF; // Max resolution of 125ms per tick
    let mut rtc = rtc::Rtc::new(dp.RTC0_NS, prescaler).unwrap();

    rtc.set_compare(
        rtc::RtcCompareReg::Compare0,
        SLEEP_MS / (1000 / (clocks::LFCLK_FREQ / (prescaler + 1))),
    )
        .unwrap();

    rtc.enable_event(rtc::RtcInterrupt::Compare0);
    rtc.enable_interrupt(rtc::RtcInterrupt::Compare0, Some(&mut cp.NVIC));
    rtc.enable_counter();

    // Place our timer in RTC mutex
    cortex_m::interrupt::free(|cs| {
        RTC.borrow(cs).replace(Some(rtc));
    });

    // State transition from Initialize to Ready
    cortex_m::interrupt::free(|cs| {
        let device = DEVICE.borrow(cs).borrow();
        if let Some(device) = device.as_ref() {
            device.state.step();
    // let state = DEVICE.unwrap().state.step();
            loop {
                #[allow(unreachable_code)]
                // cortex_m::interrupt::free(|cs| {
                // let device = DEVICE.borrow(cs).borrow();
                match device.state {
                    // Shouldn't be in the Initialize state, but just in case...
                    StateWrapper::Initialize(_) => { device.state.step(); }
                    // Low power mode when in Sleep state
                    StateWrapper::Sleep(_) => { wfe(); }
                    // Ready state means things need to wake up before we do work!
                    StateWrapper::Ready(_) => {
                        // TODO: Add code for turning on ADC/Hall Effect pin, maybe some modem stuff
                        device.state.step();
                        // TODO: Once Hall Effect is on, there needs to be at least 10 us of delay before sampling can begin
                    }
                    StateWrapper::Sample(_) => {
                         let mut sum = 0 as usize;

                         // Take 10 adc measurements and calculate mean
                         for _ in 0..11 {
                             sum += adc.read(&mut adc_pin).unwrap() as usize;
                         }

                         // Push tank level value to payload buffer, if the buffer is
                         payload_buffer.push((sum / 10) as u8).unwrap();
                            device.state.step();
                     }

                    StateWrapper::Transmit(_) => {
                        // TODO: Add code to transmit payload buffer
                        // clear buffer
                        payload_buffer.clear();
                        // Transition into Sleep state
                        device.state.step();
                    }
                }
            }
        }
    });

    unreachable!();
}

/// Interrupt handler for our timer interrupt to awake from low power mode
#[interrupt]
#[allow(non_snake_case)]
fn RTC0() {
    println!("Timer Interrupt");
    println!("Sleep time");
    cortex_m::interrupt::free(|cs| {
        let rtc = RTC.borrow(cs).borrow();
        let device = DEVICE.borrow(cs).borrow();

        // reset our timer
        if let Some(rtc) = rtc.as_ref() {
            rtc.reset_event(rtc::RtcInterrupt::Compare0);
            rtc.clear_counter();
        }
        // This should put us from Sleep state to Ready State
        if let Some(device) = device.as_ref() {
            device.state.step();
        }
    });
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
