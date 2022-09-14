#![no_main]
#![no_std]

extern crate tinyrlibc;

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm::{delay, wfe};
use cortex_m::interrupt::Mutex;
use nrf9160_hal::gpio::Level;
use nrf9160_hal::{clocks, gpio, rtc};
use defmt::println;
use nrf9160_hal::pac::{self, interrupt, RTC0_NS};
use nrf9160_hal::prelude::OutputPin;
use propane_monitor as _; // global logger + panicking-behavior + memory layout

// How long to sleep: 15 seconds
static SLEEP_MS: u32 =  60 * 1000 / 4;
// Thread safe timer
static RTC: Mutex<RefCell<Option<rtc::Rtc<RTC0_NS>>>> = Mutex::new(RefCell::new(None));
// Startup as if timer is already expired
static TIMER_EXPIRED: AtomicBool = AtomicBool::new(true);

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

    // Blue LED for the Conexio Stratus board
    // let mut blue_led = port0.p0_03.into_push_pull_output(Level::Low);

    // Disable uarte and p0_29 to reduce power consumption
    port0.p0_29.into_disconnected();
    dp.UARTE0_NS.enable.write(|w| w.enable().disabled());
    dp.UARTE1_NS.enable.write(|w| w.enable().disabled());

    // Initialize the modem, then turn off for low power
    nrfxlib::init().unwrap();
    nrfxlib::modem::set_system_mode(nrfxlib::modem::SystemMode::LteM).unwrap();
    nrfxlib::modem::on().unwrap();
    // nrfxlib::at::send_at_command(r#"AT+CPSMS=1,"","","00100001","00000000""#,|_| {}).unwrap();
    nrfxlib::modem::off().unwrap();

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

    println!("Configuration Complete");

    loop {
        // When timer is expired, replace with false.  Ordering::Relaxed means ordering constraints,
        // only atomic operations
        if TIMER_EXPIRED.compare_exchange(true, false, Ordering::Relaxed, Ordering::Relaxed)
            == Ok(true)
        {
            println!("Do work");
        }

        wfe();
    }
}

/// Interrupt handler for our timer interrupt to awake from low power mode
#[interrupt]
#[allow(non_snake_case)]
fn RTC0() {
    println!("Timer Interrupt");
    println!("Sleep time");
    cortex_m::interrupt::free(|cs| {
        let rtc = RTC.borrow(cs).borrow();

        // reset our timer
        if let Some(rtc) = rtc.as_ref() {
            rtc.reset_event(rtc::RtcInterrupt::Compare0);
            rtc.clear_counter();
        }
    });

    TIMER_EXPIRED.store(false, Ordering::Relaxed);
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
