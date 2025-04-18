#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_usb::class::cdc_ncm::{self, CdcNcmClass};
use esp_hal::delay::Delay;
use esp_hal::gpio;
use esp_hal::otg_fs::asynch::Driver;
use esp_hal::otg_fs::Usb;
use esp_hal::uart::{Config, Uart};
use esp_println::println;
use gpio::{Level, Output};
use static_cell::StaticCell;

type MyDriver = Driver<'static>;

const MTU: usize = 1514;

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

#[embassy_executor::task]
async fn usb_task(mut device: embassy_usb::UsbDevice<'static, MyDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn usb_ncm_task(class: cdc_ncm::embassy_net::Runner<'static, MyDriver, MTU>) -> ! {
    class.run().await
}

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<'static, cdc_ncm::embassy_net::Device<'static, MTU>>,
) -> ! {
    runner.run().await
}


#[embassy_executor::task]
async fn dhcp_server(stack: embassy_net::Stack<'static>) {
    use core::net::Ipv4Addr;
    let me = Ipv4Addr::new(192, 168, 2, 1);
    let mask = Ipv4Addr::new(255, 255, 255, 0);
    let config = esp_hal_dhcp_server::structs::DhcpServerConfig {
        ip: me,
        lease_time: embassy_time::Duration::from_secs(3600),
        gateways: &[me],
        subnet: Some(mask),
        dns: &[me],
        use_captive_portal: false,
    };
    let mut leaser = esp_hal_dhcp_server::simple_leaser::SimpleDhcpLeaser {
        start: Ipv4Addr::new(192, 168, 2, 50),
        end: Ipv4Addr::new(192, 168, 2, 200),
        leases: Default::default(),
    };
    esp_hal_dhcp_server::run_dhcp_server(stack, config, &mut leaser).await.unwrap();
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
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
    static NETWORK_STATE: StaticCell<cdc_ncm::State> = StaticCell::new();
    let network_state = NETWORK_STATE.init(cdc_ncm::State::new());
    let mac_address = [0xcc, 0x8d, 0xa2, 0x8d, 0x9b, 0x14];
    let network_class = CdcNcmClass::new(&mut builder, network_state, mac_address, 64);

    // Build the builder.
    let usb = builder.build();
    spawner.spawn(usb_task(usb)).unwrap();

    // Without this delay, the CDC-NCM stacks gets stuck for some reason. I have not found the
    // reason why, and will probably need JTAG to debug it properly. For now, it will do.
    // TODO: Make it work without this delay.
    Timer::after_secs(1).await;

    static ETHERNET_STATE: StaticCell<cdc_ncm::embassy_net::State<MTU, 4, 4>> = StaticCell::new();
    let ethernet_state = ETHERNET_STATE.init(cdc_ncm::embassy_net::State::new());
    let ethernet_address = [0x88, 0x88, 0x88, 0x88, 0x88, 0x88];
    let (network_runner, network_device) =
        network_class.into_embassy_net_device(ethernet_state, ethernet_address);
    spawner.spawn(usb_ncm_task(network_runner)).unwrap();

    let network_config = {
        use embassy_net::{Config, Ipv4Address, Ipv4Cidr, StaticConfigV4};
        Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 2, 1), 24),
            gateway: None,
            dns_servers: Default::default(),
        })
    };
    static NETWORK_RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();
    let network_resources = NETWORK_RESOURCES.init(embassy_net::StackResources::new());
    let (stack, runner) = embassy_net::new(
        network_device,
        network_config,
        network_resources,
        0x42424242,
    );
    spawner.spawn(net_task(runner)).unwrap();
    Timer::after_secs(1).await;

    spawner.spawn(dhcp_server(stack)).unwrap();
    Timer::after_secs(1).await;

    use embassy_net::udp::PacketMetadata;
    static RX_BUFFER: StaticCell<[u8; 4096]> = StaticCell::new();
    static TX_BUFFER: StaticCell<[u8; 4096]> = StaticCell::new();
    static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    let rx_buffer = RX_BUFFER.init([0; 4096]);
    let tx_buffer = TX_BUFFER.init([0; 4096]);
    let rx_meta = RX_META.init([PacketMetadata::EMPTY; 16]);
    let tx_meta = TX_META.init([PacketMetadata::EMPTY; 16]);
    let mut socket =
        embassy_net::udp::UdpSocket::new(stack, rx_meta, rx_buffer, tx_meta, tx_buffer);
    socket.bind(0).unwrap();
    loop {
        if stack.is_link_up() {
            let a = embassy_net::Ipv4Address::new(192, 168, 2, 50);
            println!("Sending datagram!");
            let res = socket.send_to(b"Hello, World\n", (a, 4242)).await;
            println!("{:?}", res);
            println!("Sent datagram");
        }
        Timer::after_secs(1).await;
    }
}
