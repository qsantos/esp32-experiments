#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use esp_hal::delay::Delay;
use esp_hal::gpio;
use esp_hal::otg_fs::asynch::Driver;
use esp_hal::otg_fs::Usb;
use esp_hal::uart::{Config, Uart};
use gpio::{Level, Output};
use static_cell::StaticCell;

use core::fmt::Write;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // TODO: this does not work once peripherals are initialized in main()
    let peripherals = esp_hal::init(Default::default());
    let mut uart0 = Uart::new(peripherals.UART0, Config::default())
        .unwrap()
        .with_rx(peripherals.GPIO37)
        .with_tx(peripherals.GPIO39);
    if let Some(location) = info.location() {
        write!(uart0, "panicked at {}:\r\n{}\r\n", location, info.message()).unwrap();
    } else {
        write!(
            uart0,
            "Panic at unknown location:\r\n{}\r\n",
            info.message()
        )
        .unwrap();
    }
    loop {}
}

fn blink(led: &mut Output, times: u32) {
    let delay = Delay::new();
    delay.delay_millis(1000u32);
    for _ in 0..times {
        led.toggle();
        delay.delay_millis(100u32);
        led.toggle();
        delay.delay_millis(100u32);
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(Default::default());
    let mut led = Output::new(peripherals.GPIO15, Level::Low);

    blink(&mut led, 10);

    let mut uart0 = Uart::new(peripherals.UART0, Config::default())
        .unwrap()
        .with_rx(peripherals.GPIO37)
        .with_tx(peripherals.GPIO39);
    let delay = Delay::new();
    loop {
        write!(uart0, "Hello, world!\r\n").unwrap();
        delay.delay_millis(500u32);
        led.toggle();
    }

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
    static DRIVER_BUFFER: StaticCell<[u8; 64]> = StaticCell::new();
    let config = Default::default();
    let driver = Driver::new(usb, DRIVER_BUFFER.init([0; 64]), config);

    blink(&mut led, 1);

    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("AAAAAAAAAA");
        config.product = Some("BBBBBBBBBB");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    blink(&mut led, 2);

    //// Create classes on the builder.
    //let class = {
    //    static STATE: StaticCell<State> = StaticCell::new();
    //    let state = STATE.init(State::new());
    //    CdcAcmClass::new(&mut builder, state, 64)
    //};

    // Build the builder.
    let mut usb = builder.build();

    blink(&mut led, 3);

    // Run the USB device.
    //spawner.spawn(usb_task(usb)).unwrap();
    usb.run().await;

    blink(&mut led, 4);

    loop {
        led.set_high();
        //Not working
        //Timer::after_secs(1).await;

        led.set_low();
        //Timer::after_secs(1).await;
    }
}
