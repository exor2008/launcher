#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::println;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use mavlink;
use mavlink::common::{HEARTBEAT_DATA, MavMessage};
use mavlink::{MAVLinkV2MessageRaw, MavlinkVersion, MessageData, read_v2_raw_message_async};
use rtt_target::rprintln;
use static_cell::{ConstStaticCell, StaticCell};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});
const BUF_SIZE: usize = 1024 * 10;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let (tx_pin, rx_pin, uart) = (p.PIN_4, p.PIN_5, p.UART1);

    static TX_BUF: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; BUF_SIZE])[..];
    static RX_BUF: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; BUF_SIZE])[..];
    let mut config = Config::default();
    config.baudrate = 57600;
    let uart = BufferedUart::new(uart, tx_pin, rx_pin, Irqs, tx_buf, rx_buf, config);
    let (mut tx, rx) = uart.split();

    // Create our mavlink header and heartbeat message
    let header = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    };

    let heartbeat = mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_ROCKET,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    };

    // Spawn Rx loop
    spawner.spawn(rx_task(rx)).unwrap();

    loop {
        // Write the raw heartbeat message to reduce firmware flash size (using Message::ser will be add ~70KB because
        // all *_DATA::ser methods will be add to firmware).
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_data(header, &heartbeat);
        tx.write_all(raw.raw_bytes()).await.unwrap();

        // Delay for 1 second
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
pub async fn rx_task(mut rx: BufferedUartRx) {
    loop {
        // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
        // all *_DATA::deser methods will be add to firmware).
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
            .await
            .unwrap();
        // println!("Read raw message: msg_id={}", raw.message_id());
        // println!("Indeed={}", HEARTBEAT_DATA::ID == raw.message_id());

        if raw.message_id() == HEARTBEAT_DATA::ID {
            println!("HEARTBIT");
            // let heartbeat = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
            // rprintln!("heartbeat: {:?}", heartbeat.autopilot);
        } else if raw.message_id() == 111 {
            println!("TIME_SYNC");
        }
    }
}
