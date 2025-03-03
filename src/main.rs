#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::net::IpAddr;
use core::ptr::addr_of_mut;
use core::slice;
use core::str::FromStr;

use defmt::{info, trace, unwrap, warn};
use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, Stack, StackResources};
use embassy_net_nrf91::context::Status;
use embassy_net_nrf91::{context, Runner, State, TraceBuffer, TraceReader};
use embassy_nrf::buffered_uarte::{self, BufferedUarteTx};
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive, Pin};
use embassy_nrf::uarte::Baudrate;
use embassy_nrf::{bind_interrupts, interrupt, peripherals, uarte};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod secrets;

#[interrupt]
fn IPC() {
    embassy_net_nrf91::on_ipc_irq();
}

bind_interrupts!(struct Irqs {
    SERIAL0 => buffered_uarte::InterruptHandler<peripherals::SERIAL0>;
});

#[embassy_executor::task]
async fn trace_task(
    mut uart: BufferedUarteTx<'static, peripherals::SERIAL0>,
    reader: TraceReader<'static>,
) -> ! {
    let mut rx = [0u8; 1024];
    loop {
        let n = reader.read(&mut rx[..]).await;
        unwrap!(uart.write_all(&rx[..n]).await);
    }
}

#[embassy_executor::task]
async fn modem_task(runner: Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<'static, embassy_net_nrf91::NetDriver<'static>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn control_task(
    control: &'static context::Control<'static>,
    config: context::Config<'static>,
    stack: Stack<'static>,
) {
    unwrap!(control.configure(&config).await);
    unwrap!(
        control
            .run(|status| {
                stack.set_config_v4(status_to_config(status));
            })
            .await
    );
}

fn status_to_config(status: &Status) -> embassy_net::ConfigV4 {
    let Some(IpAddr::V4(addr)) = status.ip else {
        panic!("Unexpected IP address");
    };

    let gateway = match status.gateway {
        Some(IpAddr::V4(addr)) => Some(addr),
        _ => None,
    };

    let mut dns_servers = Vec::new();
    for dns in status.dns.iter() {
        if let IpAddr::V4(ip) = dns {
            unwrap!(dns_servers.push(*ip));
        }
    }

    embassy_net::ConfigV4::Static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(addr, 32),
        gateway,
        dns_servers,
    })
}

#[embassy_executor::task]
async fn blink_task(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after_millis(1000).await;
        led.set_low();
        Timer::after_millis(1000).await;
    }
}

extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Hello World!");

    unwrap!(spawner.spawn(blink_task(p.P0_02.degrade())));

    let ipc_mem = unsafe {
        let ipc_start = &__start_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_end = &__end_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_len = ipc_end.offset_from(ipc_start) as usize;
        slice::from_raw_parts_mut(ipc_start, ipc_len)
    };

    info!("Past IPC");

    static mut TRACE_BUF: [u8; 4096] = [0u8; 4096];
    let mut config = uarte::Config::default();
    config.baudrate = Baudrate::BAUD1M;
    let uart = BufferedUarteTx::new(
        //let trace_uart = BufferedUarteTx::new(
        unsafe { peripherals::SERIAL0::steal() },
        Irqs,
        unsafe { peripherals::P0_01::steal() },
        //unsafe { peripherals::P0_14::steal() },
        config,
        unsafe { &mut *addr_of_mut!(TRACE_BUF) },
    );
    info!("Past UART");

    static STATE: StaticCell<State> = StaticCell::new();
    static TRACE: StaticCell<TraceBuffer> = StaticCell::new();
    let (device, control, runner, tracer) = embassy_net_nrf91::new_with_trace(
        STATE.init(State::new()),
        ipc_mem,
        TRACE.init(TraceBuffer::new()),
    )
    .await;
    info!("Past modem 1");
    unwrap!(spawner.spawn(modem_task(runner)));
    info!("Past modem 2");
    unwrap!(spawner.spawn(trace_task(uart, tracer)));
    info!("Past modem 3");
    trace!("Test");

    let config = embassy_net::Config::default();

    // Generate "random" seed. nRF91 has no RNG, TODO figure out something...
    let seed = 123456;

    // Init network stack
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        config,
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    );
    info!("Past resources");

    unwrap!(spawner.spawn(net_task(runner)));
    info!("Past net task runner");

    static CONTROL: StaticCell<context::Control<'static>> = StaticCell::new();
    let control = CONTROL.init(context::Control::new(control, 0).await);

    info!("Sim connecting");

    unwrap!(spawner.spawn(control_task(
        control,
        context::Config {
            apn: b"eapn1.net ",
            auth_prot: context::AuthProt::Chap,
            auth: None,
            pin: Some(&secrets::SIM_PIN),
        },
        stack
    )));
    info!("Sim conf done");

    stack.wait_config_up().await;
    info!("Sim up!");

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        info!("Connecting...");
        //let host_addr = embassy_net::Ipv4Address::from_str("45.79.112.203").unwrap();
        let host_addr = embassy_net::Ipv4Address::from_str("129.241.152.48").unwrap();
        //if let Err(e) = socket.connect((host_addr, 4242)).await {
        if let Err(e) = socket.connect((host_addr, 9000)).await {
            warn!("connect error: {:?}", e);
            socket.close();
            Timer::after_secs(10).await;
            continue;
        }
        info!("Connected to {:?}", socket.remote_endpoint());

        let msg = b"Hello world!\n";
        let mut read_buf: [u8; 256] = [0u8; 256];
        for _ in 0..10 {
            if let Err(e) = socket.write_all(msg).await {
                warn!("write error: {:?}", e);
                break;
            }
            info!("txd: {}", core::str::from_utf8(msg).unwrap());
            if let Err(e) = socket.read(&mut read_buf).await {
                warn!("Read error: {:?}", e);
                break;
            }
            info!("rxd: {}", read_buf);
            read_buf = [0u8; 256];
            Timer::after_secs(1).await;
        }
        socket.close();
        Timer::after_secs(4).await;
    }
}
