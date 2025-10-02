#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartTx, Config};
use embassy_time::Timer;
use embedded_io_async::Write;
use mavlink;
use mavlink::ardupilotmega::{
    COMMAND_ACK_DATA, COMMAND_LONG_DATA, HEARTBEAT_DATA, MavMessage, RC_CHANNELS_DATA,
    SERVO_OUTPUT_RAW_DATA,
};
use mavlink::{MAVLinkV2MessageRaw, MavlinkVersion, MessageData, read_v2_raw_message_async};
// use rtt_target::{rprintln, rtt_init_print};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});
const BUF_SIZE: usize = 1024 * 10;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // rtt_init_print!();

    let p = embassy_rp::init(Default::default());

    let (tx_pin, rx_pin, uart) = (p.PIN_4, p.PIN_5, p.UART1);

    static TX_BUF: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; BUF_SIZE])[..];
    static RX_BUF: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; BUF_SIZE])[..];
    let mut config = Config::default();

    config.baudrate = 57600;
    let uart = BufferedUart::new(uart, tx_pin, rx_pin, Irqs, tx_buf, rx_buf, config);
    let (tx, mut rx) = uart.split();

    info!("Waiting for the heartbeat...");
    loop {
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
            .await
            .unwrap();

        if raw.message_id() == HEARTBEAT_DATA::ID {
            info!("Got heartbeat, starting.");
            break;
        }
    }

    // Spawn Tx loop
    spawner.spawn(tx_task(tx)).unwrap();

    loop {
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
            .await
            .unwrap();

        match raw.message_id() {
            HEARTBEAT_DATA::ID => {
                // info!("HEARTBEAT");
                // let hb = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
                // rprintln!("heartbeat: {:?}", hb);
            }
            SERVO_OUTPUT_RAW_DATA::ID => {
                let servo =
                    SERVO_OUTPUT_RAW_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
                info!("SERVO port: {}", servo.port);
            }
            RC_CHANNELS_DATA::ID => {
                let rc = RC_CHANNELS_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
                info!("RC9: {}", rc.chan9_raw);
            }
            COMMAND_ACK_DATA::ID => {
                let ack = COMMAND_ACK_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
                info!(
                    "ACK Cmd: {}, Result: {}",
                    ack.command as u16, ack.result as u8
                );
            }
            _ => info!("Message id: {}", raw.message_id()),
        }
    }
}

#[embassy_executor::task]
pub async fn tx_task(mut tx: BufferedUartTx) {
    // Requesting the servo data
    info!("Sending Servo request");
    write_mav(&mut tx, &servo_data()).await;

    loop {
        write_mav(&mut tx, &heartbeat_data()).await;

        // Delay for 1 second
        Timer::after_millis(1000).await;
    }
}

async fn write_mav<M>(tx: &mut BufferedUartTx, msg: &M)
where
    M: MessageData,
{
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message_data(header(), msg);
    tx.write_all(raw.raw_bytes()).await.unwrap();
}

fn header() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 42,
        component_id: mavlink::ardupilotmega::MavComponent::MAV_COMP_ID_PERIPHERAL as u8 + 13,
        sequence: 0,
    }
}

fn heartbeat_data() -> HEARTBEAT_DATA {
    HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_SERVO,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    }
}

fn servo_data() -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        // param1: ACTUATOR_OUTPUT_STATUS_DATA::ID as f32,
        // param1: mavlink::common::BATTERY_STATUS_DATA::ID as f32,
        // param1: SERVO_OUTPUT_RAW_DATA::ID as f32,
        param1: RC_CHANNELS_DATA::ID as f32,
        // param1: 211.0,
        param2: 1_000_00.0,
        // param4: 1.0,
        confirmation: 0,
        ..Default::default()
    }
}
