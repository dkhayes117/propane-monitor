#![no_main]
#![no_std]

// links in a minimal version of libc
extern crate tinyrlibc;

use core::cell::RefCell;

use cortex_m::asm::{delay, wfe};
use cortex_m::interrupt::Mutex;

use defmt::{println, unwrap};

use nrf9160_hal::pac::RTC0_NS;
use nrf9160_hal::saadc::SaadcConfig;
use nrf9160_hal::{
    clocks,
    gpio,
    pac::{self, interrupt},
    prelude::*,
    // pwm,
    // pwm::Pwm,
    rtc,
    Saadc,
};

use crate::gpio::{Level, OpenDrainConfig};
// use nrf_modem_nal::embedded_nal::{heapless, SocketAddr, UdpClientStack};
use propane_monitor as _; // global logger + panicking-behavior + memory layout
use propane_monitor::{Event, Payload, State};

// const MILLISECOND_CYCLES: u32 = nrf9160_hal::Timer::<pac::TIMER0_NS>::TICKS_PER_SECOND / 1000;

// How long to sleep: 15 seconds
static SLEEP_MS: u32 = 15_000;

// A static buffer to hold data between data transfers which lives on the stack
// Mutex and RefCell are to ensure safe mutability
// static mut PAYLOAD_BUFFER: Mutex<RefCell<Vec<u8, 6>>> = Mutex::new(RefCell::new(Vec::new()));

// Thread safe timer
static RTC: Mutex<RefCell<Option<rtc::Rtc<RTC0_NS>>>> = Mutex::new(RefCell::new(None));
// Global state variable which begins in the 'Initialize' state
static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::Initialize));

#[cortex_m_rt::entry]
fn main() -> ! {
    // Take ownership of core and device peripherals
    let mut cp = unwrap!(cortex_m::Peripherals::take());
    let dp = unwrap!(pac::Peripherals::take());

    // A static buffer to hold data between data transfers which lives on the stack
    let mut payload = Payload::new();

    // Get handle for port0 GPIO
    let port0 = gpio::p0::Parts::new(dp.P0_NS);

    // Disable uarte0, uarte1, and p0_29 and leave off to reduce power consumption
    port0.p0_29.into_disconnected();
    dp.UARTE0_NS.enable.write(|w| w.enable().disabled());
    dp.UARTE1_NS.enable.write(|w| w.enable().disabled());

    // Configuration for reading hall effect voltage output
    // Default has Oversampling = 8 (2^3) times
    let adc_config = SaadcConfig::default();
    let mut adc = Saadc::new(dp.SAADC_NS, adc_config);

    // Icarus uses the P0.13 pin internally for battery voltage measurement
    // P0.14 works on both Icarus and Stratus boards, or replace with a usable analog pin on your board
    let mut adc_pin = port0.p0_14.into_floating_input();
    // Config a power pin for the hall effect sensor so it can be powered down when not sampling
    // let mut hall_effect_power = port0.p0_31.into_disconnected();
    let mut hall_effect_power = port0
        .p0_31
        .into_open_drain_output(OpenDrainConfig::Disconnect0HighDrive1, Level::Low);

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
    nrfxlib::init().unwrap();
    nrfxlib::modem::set_system_mode(nrfxlib::modem::SystemMode::LteM).unwrap();
    nrfxlib::modem::on().unwrap();
    // nrfxlib::at::send_at_command(r#"AT+CPSMS=1,"","","00100001","00000000""#,|_| {}).unwrap();
    nrfxlib::modem::off().unwrap();

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

    // Place our timer in RTC mutex, step our state machine to READY
    cortex_m::interrupt::free(|cs| {
        RTC.borrow(cs).replace(Some(rtc));
        let mut state = *STATE.borrow(cs).borrow();

        println!("{:?}", state);
        state = state.step(Event::SetupComplete);
        STATE.borrow(cs).replace(state);
    });

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut state = *STATE.borrow(cs).borrow();
            match state {
                // Shouldn't be in the Initialize state, but just in case...
                State::Initialize => {
                    println!("{:?}", state);
                    state = state.step(Event::SetupComplete);
                }

                // Low power mode when in Sleep state
                State::Sleep => {
                    // Do nothing until timer interrupt occurs
                    wfe();
                    println!("{:?}", state);
                }

                // Ready state means things need to wake up before we do work!
                State::Ready => {
                    // TODO: turn on modem
                    hall_effect_power.set_high().unwrap();

                    // Hall effect sensor as as a power up time of 330us max, 175us typical
                    // 32_000_000 cycles * 330us = 10_560 cycles
                    delay(15_000);

                    println!("{:?}", state);
                    state = state.step(Event::SensorPowerOn);
                }

                State::Sample => {
                    let value = adc.read(&mut adc_pin).unwrap();
                    let mut event = Event::BufferNotFull;

                    println!("{:?}, {}", state, value);

                    // Turn off the hall sensor when done sampling
                    hall_effect_power.set_low().unwrap();

                    // Push tank level value to payload buffer, if the buffer is full then transmit
                    payload.data.push(value).unwrap();

                    if payload.data.is_full() {
                        event = Event::BufferFull;
                        println!("Buffer is Full!");
                    };

                    state = state.step(event);
                }

                State::Transmit => {
                    // TODO: Add code to transmit payload buffer

                    // clear buffer after transmission
                    payload.data.clear();

                    // Transition into Sleep state
                    println!("{:?}", state);
                    state = state.step(Event::DataSent);
                }

                State::Failure(_error) => {
                    println!("{:?}", state);
                    // TODO: Add code to reset the device back into a good state
                    propane_monitor::exit();
                }
            }
            // Update our global state
            STATE.borrow(cs).replace(state);
        });
    }
}

/// Interrupt handler for our timer interrupt to awake from low power mode
#[interrupt]
#[allow(non_snake_case)]
fn RTC0() {
    println!("Timer Interrupt");
    cortex_m::interrupt::free(|cs| {
        let rtc = RTC.borrow(cs).borrow();
        let mut state = *STATE.borrow(cs).borrow();

        // reset our timer
        if let Some(rtc) = rtc.as_ref() {
            rtc.reset_event(rtc::RtcInterrupt::Compare0);
            rtc.clear_counter();
        }

        // The line below is an undefined transition which forces us in the Failure state
        state = state.step(Event::DataSent);
        // This should put us from Sleep state to Ready State
        // state = state.step(Event::TimerInterrupt);
        STATE.borrow(cs).replace(state);
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
