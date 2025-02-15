#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Io, Level, Output, GpioPin};
use esp_hal::main;
use log::info;

extern crate alloc;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let mut led = Output::new(peripherals.GPIO15, Level::High);
    let delay = Delay::new();
    loop {
        led.toggle();
        delay.delay_millis(1000);
    }
}
