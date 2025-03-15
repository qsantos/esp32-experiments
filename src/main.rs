#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{self, CdcAcmClass};
use embassy_usb::class::hid::{self, HidWriter};
use embassy_usb::driver::EndpointError;
use esp_hal::delay::Delay;
use esp_hal::gpio;
use esp_hal::otg_fs::asynch::Driver;
use esp_hal::otg_fs::Usb;
use esp_hal::uart::{Config, Uart};
use esp_println::println;
use gpio::{Level, Output};
use usbd_hid::descriptor::{KeyboardReport, KeyboardUsage, SerializedDescriptor};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("{}", info);
    println!("Resetting");
    esp_hal::reset::software_reset();
    loop {}
}

fn blink(led: &mut Output, times: u32) {
    let delay = Delay::new();
    if times != 0 {
        delay.delay_millis(100u32);
        led.toggle();
        delay.delay_millis(100u32);
    }
    for _ in 1..times {
        led.toggle();
        delay.delay_millis(100u32);
        led.toggle();
        delay.delay_millis(100u32);
    }
}

async fn echo_loop<'a>(
    class: &mut cdc_acm::CdcAcmClass<'a, Driver<'a>>,
) -> Result<(), EndpointError> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        buf.make_ascii_uppercase();
        class.write_packet(&buf[..n]).await?;
    }
}

#[repr(u8)]
#[allow(unused)]
enum KeyboardModifier {
    LeftControl = 0x01,
    LeftShift = 0x02,
    LeftAlt = 0x04,
    LeftGui = 0x08,
    RightControl = 0x10,
    RightShift = 0x20,
    RightAlt = 0x40,
    RightGui = 0x80,
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(Default::default());
    // set UART0 first thing, we can send logs and panic messages
    Uart::new(peripherals.UART0, Config::default())
        .unwrap()
        .with_rx(peripherals.GPIO37)
        .with_tx(peripherals.GPIO39);

    // enable logging
    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let mut led = Output::new(peripherals.GPIO15, Level::Low);
    blink(&mut led, 1);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
    let mut driver_buffer = [0; 1024];
    let config = Default::default();
    let driver = Driver::new(usb, &mut driver_buffer, config);

    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("AAAAAAAAAA");
        config.product = Some("BBBBBBBBBB");
        config.serial_number = Some("12345678");
        config
    };

    // We need to declare the state before the builder to ensure it is dropped after the builder.
    let mut echo_state = cdc_acm::State::new();
    let mut hid_state = hid::State::new();

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut echo_class = CdcAcmClass::new(&mut builder, &mut echo_state, 64);
    let mut hid_class: HidWriter<_, 16> = HidWriter::new(
        &mut builder,
        &mut hid_state,
        hid::Config {
            report_descriptor: KeyboardReport::desc(),
            request_handler: None,
            poll_ms: 10,
            max_packet_size: 64,
        },
    );

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let echo_fut = async {
        loop {
            echo_class.wait_connection().await;
            println!("Connected");
            if let Err(EndpointError::BufferOverflow) = echo_loop(&mut echo_class).await {
                panic!("Buffer overflow");
            }
            println!("Disconnected");
        }
    };

    let hid_fut = async {
        loop {
            // press A
            let mut report = KeyboardReport::default();
            report.keycodes[0] = KeyboardUsage::KeyboardAa as u8;
            report.modifier |= KeyboardModifier::LeftShift as u8;
            if let Err(EndpointError::BufferOverflow) = hid_class.write_serialize(&report).await {
                panic!("Buffer overflow");
            }
            // release A
            let report = KeyboardReport::default();
            if let Err(EndpointError::BufferOverflow) = hid_class.write_serialize(&report).await {
                panic!("Buffer overflow");
            }
            Timer::after_secs(1).await;
        }
    };

    let blinker = async {
        loop {
            led.set_high();
            Timer::after_secs(1).await;

            led.set_low();
            Timer::after_secs(1).await;
        }
    };

    join4(usb_fut, echo_fut, blinker, hid_fut).await;
}
