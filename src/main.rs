#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join5;
use embassy_net;
use embassy_time::Timer;
use embassy_usb::class::cdc_ncm::{self, CdcNcmClass};
use embassy_usb::driver::EndpointError;
use esp_hal::delay::Delay;
use esp_hal::gpio;
use esp_hal::otg_fs::asynch::Driver;
use esp_hal::otg_fs::Usb;
use esp_hal::uart::{Config, Uart};
use esp_println::println;
use gpio::{Level, Output};
use static_cell::StaticCell;

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
        led.toggle();
        delay.delay_millis(100u32);
        led.toggle();
    }
    for _ in 1..times {
        delay.delay_millis(100u32);
        led.toggle();
        delay.delay_millis(100u32);
        led.toggle();
    }
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
    static DRIVER_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let driver_buffer = DRIVER_BUFFER.init([0; 1024]);
    let config = Default::default();
    let driver = Driver::new(usb, driver_buffer, config);

    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("AAAAAAAAAA");
        config.product = Some("BBBBBBBBBB");
        config.serial_number = Some("12345678");
        config
    };

    // We need to declare the state before the builder to ensure it is dropped after the builder.
    let mut network_state = cdc_ncm::State::new();
    static ETHERNET_STATE: StaticCell<cdc_ncm::embassy_net::State<1514, 4, 4>> = StaticCell::new();
    let ethernet_state = ETHERNET_STATE.init(cdc_ncm::embassy_net::State::new());

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create classes on the builder.
    let mac_address = [0xcc, 0x8d, 0xa2, 0x8d, 0x9b, 0x14];
    let network_class = CdcNcmClass::new(&mut builder, &mut network_state, mac_address, 64);
    let ethernet_address = [0xcc, 0x8d, 0xa2, 0x8d, 0x9b, 0x14];
    let (network_runner, network_device) =
        network_class.into_embassy_net_device(ethernet_state, ethernet_address);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();
    let network_runner_fut = network_runner.run();

    let network_config = {
        use embassy_net::{Config, Ipv4Address, Ipv4Cidr, StaticConfigV4};
        Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 1, 2), 24),
            gateway: None,
            dns_servers: Default::default(),
        })
    };
    static NETWORK_RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();
    let network_resources = NETWORK_RESOURCES.init(embassy_net::StackResources::new());
    let (stack, mut runner) = embassy_net::new(
        network_device,
        network_config,
        network_resources,
        0x42424242,
    );
    let stack_fut = runner.run();
    let udp_fut = async {
        use embassy_net::udp::PacketMetadata;
        let mut rx_buffer = [0; 10];
        let mut tx_buffer = [0; 10];
        let mut rx_meta = [PacketMetadata::EMPTY; 2];
        let mut tx_meta = [PacketMetadata::EMPTY; 2];
        let mut socket = embassy_net::udp::UdpSocket::new(
            stack,
            &mut rx_meta,
            &mut rx_buffer,
            &mut tx_meta,
            &mut tx_buffer,
        );
        socket.bind(0).unwrap();
        loop {
            let a = embassy_net::Ipv4Address::new(192, 168, 1, 42);
            println!("Sending datagram!");
            let res = socket.send_to(b"Hello, World", (a, 4242)).await;
            println!("{:?}", res);
            println!("Sent datagram");
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

    join5(usb_fut, blinker, network_runner_fut, stack_fut, udp_fut).await;
}
