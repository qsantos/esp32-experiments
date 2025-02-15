use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;

use log::info;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let mut led = PinDriver::output(peripherals.pins.gpio15)?;

    loop {
        info!("Hello, world!");
        led.set_high()?;
        FreeRtos::delay_ms(500);
        led.set_low()?;
        FreeRtos::delay_ms(500);
    }
}
