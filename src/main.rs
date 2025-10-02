#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::sync::atomic::Ordering;

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartTx, Config};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embedded_io_async::Write;
use launcher::{FireControl, FireDetector};
use mavlink;
use mavlink::ardupilotmega::{
    COMMAND_ACK_DATA, COMMAND_LONG_DATA, HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage,
    MavModeFlag, MavResult, MavState, MavType, RC_CHANNELS_DATA,
};
use mavlink::{MAVLinkV2MessageRaw, MavlinkVersion, MessageData, read_v2_raw_message_async};
use portable_atomic::AtomicBool;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

const BUF_SIZE: usize = 1024 * 10;
const ATEMPTS: usize = 100;
const WAIT_FC_START: u64 = 3; // sec
const HB_SEND_INTERVAL: u64 = 1; // sec
const HB_WATCHDOG_INTERVAL: u64 = 2; // sec
const LAUNCHERS: usize = 8; // sec

pub static IS_NEW_HB: AtomicBool = AtomicBool::new(true);
static FIRE_CHANNEL: Channel<ThreadModeRawMutex, bool, LAUNCHERS> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // UART
    let (tx_pin, rx_pin, uart) = (p.PIN_4, p.PIN_5, p.UART1);

    static TX_BUF: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; BUF_SIZE])[..];
    static RX_BUF: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; BUF_SIZE])[..];
    let mut config = Config::default();

    config.baudrate = 57600;
    let mut uart = BufferedUart::new(uart, tx_pin, rx_pin, Irqs, tx_buf, rx_buf, config);

    // Wait for the FC to start
    // Timer::after_secs(WAIT_FC_START).await;

    // Send first heartbeat
    info!("Sending initial Heartbeat");
    write_mav(&mut uart, &heartbeat_data()).await;

    // Wait for the first Heartbeat
    info!("Waiting for the heartbeat...");
    let mut atempts = ATEMPTS;
    loop {
        let raw = read_v2_raw_message_async::<MavMessage>(&mut uart)
            .await
            .unwrap();

        if raw.message_id() == HEARTBEAT_DATA::ID {
            info!("Got heartbeat, starting.");
            break;
        }

        atempts -= 1;

        if atempts == 0 {
            error!("No heartbeat received after {} atempts", ATEMPTS);
            panic!("No heartbeat received after {} atempts", ATEMPTS);
        }
    }

    // Send RC request
    info!("Requesting the RC...");
    write_mav(&mut uart, &rc_request_data()).await;

    // Wait for the ACK
    atempts = ATEMPTS;
    loop {
        let raw = read_v2_raw_message_async::<MavMessage>(&mut uart)
            .await
            .unwrap();

        if raw.message_id() == COMMAND_ACK_DATA::ID {
            let ack = COMMAND_ACK_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
            if ack.command as u32 == MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL as u32 {
                match ack.result {
                    MavResult::MAV_RESULT_ACCEPTED => {
                        info!("RC request accepted");
                        break;
                    }
                    _ => {
                        error!("RC request denied, error code: {}", ack.result as u8);
                        panic!("RC request denied, error code: {}", ack.result as u8);
                    }
                }
            }
        }
    }

    atempts -= 1;

    if atempts == 0 {
        error!("No heartbeat received after {} atteempts", ATEMPTS);
        panic!("No heartbeat received after {} atteempts", ATEMPTS);
    }

    // Split uart
    let (tx, mut rx) = uart.split();

    // Spawn Heartbeat Watchdog
    spawner.spawn(heartbeat_watchdog_task()).unwrap();

    // Spawn Heartbeat Sender
    spawner.spawn(send_heartbeat_task(tx)).unwrap();

    let mut fd = FireDetector::default();
    let fc: FireControl<LAUNCHERS> = FireControl::default();

    // Spawn Fire Control Task
    spawner.spawn(fire_control_task(fc)).unwrap();

    info!("Receiving RC data...");
    loop {
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
            .await
            .unwrap();

        match raw.message_id() {
            HEARTBEAT_DATA::ID => {
                IS_NEW_HB.store(true, Ordering::Relaxed);
            }
            RC_CHANNELS_DATA::ID => {
                // Send to a State machine
                let rc = RC_CHANNELS_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
                if fd.tick(rc) {
                    FIRE_CHANNEL.send(true).await;
                }
            }
            _ => info!("Message id: {}", raw.message_id()),
        }
    }
}

#[embassy_executor::task]
async fn send_heartbeat_task(mut tx: BufferedUartTx) {
    info!("Start sending heartbeats to a FC...");

    loop {
        write_mav(&mut tx, &heartbeat_data()).await;

        Timer::after_secs(HB_SEND_INTERVAL).await;
    }
}

#[embassy_executor::task]
async fn heartbeat_watchdog_task() {
    info!("Start watching heartbeats from a FC...");

    loop {
        let is_new = IS_NEW_HB.load(Ordering::Relaxed);
        match is_new {
            true => IS_NEW_HB.store(false, Ordering::Relaxed),
            false => {
                error!("No heartbeat from the Flight Controller");
                panic!("No heartbeat from the Flight Controller");
            }
        }
        Timer::after_secs(HB_WATCHDOG_INTERVAL).await;
    }
}

#[embassy_executor::task]
async fn fire_control_task(mut fc: FireControl<LAUNCHERS>) {
    info!("Start fire control task");
    loop {
        FIRE_CHANNEL.receive().await;
        fc.fire().await;
    }
}

async fn write_mav<M, W>(tx: &mut W, msg: &M)
where
    M: MessageData,
    W: Write,
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
        mavtype: MavType::MAV_TYPE_SERVO,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    }
}

fn rc_request_data() -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
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
