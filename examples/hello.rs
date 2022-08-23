#![no_main]
#![no_std]

use propane_monitor as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    propane_monitor::exit();
}