//! protocol_auditor_firmware: STM32F401RE Embassy-based protocol auditor.
//! Receives campaign config over UART (ST-LINK VCP), emits faulted sensor frames
//! and ground-truth FAULT_EVENT frames.
//!
//! The UART is asynchronous and DMA-backed. All transmissions go through one
//! queue so campaign output and control replies (PONG/END) cannot interleave
//! bytes on the wire.

#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::usart::{self, Config as UartConfig, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, dma, mode::Async, peripherals, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use panic_probe as _;

use protocol_auditor_firmware::protocol::{
    decode_start, encode_end, encode_fault_event, encode_pong, encode_sensor, CampaignConfig,
    Decoded, FrameParser, Lcg, FAULT_BITFLIP, FAULT_BURST, FAULT_BYTE, FAULT_DELAY, FAULT_DROP,
    FAULT_DUTY, FAULT_REPLAY, K_ABORT, K_FAULT, K_PING, K_START, MAX_FRAME, N_FAULT_KINDS,
};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
});

/// A complete wire frame plus its valid length.
type OutFrame = ([u8; MAX_FRAME], usize);

/// Serialises every producer that wants to write to USART2. The campaign task
/// and the UART receiver (PONG replies) both enqueue here.
static OUT: Channel<CriticalSectionRawMutex, OutFrame, 16> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("=== protocol-auditor-firmware starting ===");
    info!("Target: STM32F401RE (Nucleo-F401RE)");

    // Clock config: 84 MHz from HSI via PLL.
    let mut config = Config::default();
    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
        prediv: embassy_stm32::rcc::PllPreDiv::Div8,
        mul: embassy_stm32::rcc::PllMul::Mul168,
        divp: Some(embassy_stm32::rcc::PllPDiv::Div4), // 84 MHz
        divq: None,
        divr: None,
    });
    config.rcc.pll_src = embassy_stm32::rcc::PllSource::Hsi;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::Pll1P;
    config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::Div2;
    config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::Div1;

    let p = embassy_stm32::init(config);

    // USART2 on PA2/PA3 at 115200 (ST-LINK VCP). DMA1 stream 6 is USART2_TX
    // and stream 5 is USART2_RX on the STM32F401.
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart = Uart::new(
        p.USART2, p.PA2, p.PA3, p.DMA1_CH6, p.DMA1_CH5, Irqs, uart_cfg,
    )
    .unwrap();
    let (tx, rx) = uart.split();

    // Shared state between Rx and Tx tasks.
    static CAMPAIGN: Signal<CriticalSectionRawMutex, CampaignConfig> = Signal::new();
    static ABORT: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    static GLITCH: Signal<CriticalSectionRawMutex, ()> = Signal::new();

    spawner.spawn(uart_tx_task(tx, &OUT).unwrap());
    spawner.spawn(tx_task(&OUT, &CAMPAIGN, &ABORT, &GLITCH).unwrap());
    spawner.spawn(rx_task(rx, &OUT, &CAMPAIGN, &ABORT, &GLITCH).unwrap());

    info!("=== Firmware ready ===");
}

/// Drains the outgoing-frame queue into USART2. Keeping the UART owner in one
/// task means no two tasks can interleave fragments of different frames.
#[embassy_executor::task]
async fn uart_tx_task(
    mut tx: UartTx<'static, Async>,
    out: &'static Channel<CriticalSectionRawMutex, OutFrame, 16>,
) {
    loop {
        let (frame, n) = out.receive().await;
        if tx.write(&frame[..n]).await.is_err() {
            warn!("UART TX error");
        }
    }
}

/// Transmitter task: runs the campaign, emits faulted frames + ground-truth events.
#[embassy_executor::task]
async fn tx_task(
    out: &'static Channel<CriticalSectionRawMutex, OutFrame, 16>,
    campaign_signal: &'static Signal<CriticalSectionRawMutex, CampaignConfig>,
    abort_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    glitch_signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    let mut cfg: Option<CampaignConfig> = None;
    let mut rng = Lcg::new(0);
    let mut seq: u16 = 0;
    let mut fault_id: u32 = 0;
    let mut sent: u16 = 0;

    loop {
        // Check for new campaign.
        if let Some(new_cfg) = campaign_signal.try_take() {
            cfg = Some(new_cfg);
            rng = Lcg::new(new_cfg.seed);
            seq = 0;
            fault_id = 0;
            sent = 0;
            info!(
                "Campaign started: seed={}, packets={}, cadence={}ms",
                new_cfg.seed, new_cfg.packets, new_cfg.cadence_ms
            );
        }

        // Check for abort.
        if abort_signal.try_take().is_some() {
            cfg = None;
            info!("Campaign aborted");
        }

        // Check for duty glitch request. The physical PWM path is not wired yet.
        if glitch_signal.try_take().is_some() {
            info!("Duty glitch fired");
        }

        if let Some(c) = cfg.as_mut() {
            if sent >= c.packets {
                let total = c.packets;
                out.send(encode_end(total, fault_id)).await;
                info!("Campaign completed: {} packets, {} faults", total, fault_id);
                cfg = None;
                continue;
            }

            // Pick fault for this packet.
            let kind = pick_fault(&mut rng, &c.weights);
            let this_seq = seq;
            seq = seq.wrapping_add(1);
            sent += 1;

            let value = 100u32 + rng.below(50);
            match kind {
                FAULT_BITFLIP | FAULT_BYTE => {
                    fault_id += 1;
                    let fid = fault_id;
                    let mut payload = [0u8; 10];
                    payload[2..6].copy_from_slice(&value.to_le_bytes());
                    if kind == FAULT_BITFLIP {
                        let bit = rng.below(16) as usize;
                        payload[2 + (bit >> 3)] ^= 1 << (bit & 7);
                    } else {
                        let idx = 2 + rng.below(4) as usize;
                        payload[idx] ^= 0xA5;
                    }
                    out.send(encode_sensor(
                        this_seq,
                        u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]),
                        fid,
                    ))
                    .await;
                    out.send(encode_fault_event(fid, kind, this_seq)).await;
                }
                FAULT_DROP => {
                    fault_id += 1;
                    let fid = fault_id;
                    out.send(encode_fault_event(fid, kind, this_seq)).await;
                }
                FAULT_DELAY => {
                    fault_id += 1;
                    let fid = fault_id;
                    let delay_ms = u64::from(c.cadence_ms.max(1)) * 6;
                    Timer::after_millis(delay_ms).await; // 6x cadence = late
                    out.send(encode_sensor(this_seq, value, 0)).await;
                    out.send(encode_fault_event(fid, kind, this_seq)).await;
                }
                FAULT_REPLAY => {
                    fault_id += 1;
                    let fid = fault_id;
                    out.send(encode_sensor(this_seq, value, 0)).await;
                    out.send(encode_sensor(this_seq, value, 0)).await;
                    out.send(encode_fault_event(fid, kind, this_seq)).await;
                }
                FAULT_BURST => {
                    fault_id += 1;
                    let fid = fault_id;
                    let mut junk = [0u8; 6];
                    for j in junk.iter_mut() {
                        *j = rng.next_u8();
                    }
                    // Send raw garbage through the same serialised queue.
                    let mut junk_frame = [0u8; MAX_FRAME];
                    junk_frame[..junk.len()].copy_from_slice(&junk);
                    out.send((junk_frame, junk.len())).await;
                    out.send(encode_sensor(this_seq, value, 0)).await;
                    out.send(encode_fault_event(fid, kind, this_seq)).await;
                }
                FAULT_DUTY => {
                    fault_id += 1;
                    let fid = fault_id;
                    // Physical glitch would go here.
                    out.send(encode_sensor(this_seq, value, 0)).await;
                    out.send(encode_fault_event(fid, kind, this_seq)).await;
                }
                _ => {
                    // Clean.
                    out.send(encode_sensor(this_seq, value, 0)).await;
                }
            }

            // Clamp zero to one tick so a malformed START cannot create a
            // busy-loop that starves the executor and the receiver.
            Timer::after_millis(u64::from(c.cadence_ms.max(1))).await;
        } else {
            Timer::after_millis(10).await;
        }
    }
}

fn pick_fault(rng: &mut Lcg, weights: &[u8; N_FAULT_KINDS]) -> u8 {
    let total: u32 = weights.iter().map(|&w| w as u32).sum();
    if total == 0 {
        return 0;
    }
    let mut r = rng.below(total);
    for (i, &w) in weights.iter().enumerate() {
        if r < w as u32 {
            return (i as u8) + 1;
        }
        r -= w as u32;
    }
    0
}

/// Receiver task: parses control frames (START, ABORT, PING, GLITCH).
#[embassy_executor::task]
async fn rx_task(
    mut rx: UartRx<'static, Async>,
    out: &'static Channel<CriticalSectionRawMutex, OutFrame, 16>,
    campaign_signal: &'static Signal<CriticalSectionRawMutex, CampaignConfig>,
    abort_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    _glitch_signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 64];

    loop {
        let n = match rx.read_until_idle(&mut buf).await {
            Ok(0) => continue,
            Ok(n) => n,
            Err(_) => {
                warn!("UART RX error");
                Timer::after_millis(1).await;
                continue;
            }
        };

        for &byte in &buf[..n] {
            let Some(decoded) = parser.push(byte) else {
                continue;
            };

            match decoded {
                Decoded::Frame { kind, payload, len } => match kind {
                    K_PING => {
                        // PONG must actually go back on the wire; the host uses
                        // it as a liveness check before starting a campaign.
                        out.send(encode_pong()).await;
                        info!("PING received");
                    }
                    K_START => match decode_start(&payload[..len]) {
                        Some(cfg) => campaign_signal.signal(cfg),
                        None => warn!("START payload has invalid length"),
                    },
                    K_ABORT => abort_signal.signal(()),
                    K_FAULT => {
                        // Ground-truth frames are firmware -> host only.
                    }
                    _ => warn!("unknown command kind 0x{:02x}", kind),
                },
                Decoded::BadCrc { kind } => warn!("bad CRC, kind=0x{:02x}", kind),
                Decoded::Garbage => {
                    // Expected while the parser hunts for the next sync byte.
                }
            }
        }
    }
}
