//! faultforge-firmware: STM32F401RE Embassy-based fault injector.
//! Receives campaign config over UART (ST-LINK VCP), emits faulted sensor frames
//! and ground-truth FAULT_EVENT frames.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use panic_probe as _;

use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_time::Timer;
use faultforge_firmware::protocol::{
    decode_start, encode_end, encode_fault_event, encode_pong, encode_sensor, CampaignConfig, Lcg,
    FAULT_BITFLIP, FAULT_BURST, FAULT_BYTE, FAULT_DELAY, FAULT_DROP, FAULT_DUTY, FAULT_REPLAY,
    K_ABORT, K_FAULT, K_PING, K_START, N_FAULT_KINDS,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("=== faultforge-firmware starting ===");
    info!("Target: STM32F401RE (Nucleo-F401RE)");

    // Clock config: 84 MHz from HSI via PLL (same as embassy-telemetry-tool)
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

    // UART2 on PA2/PA3 at 115200 (ST-LINK VCP)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart = Uart::new_blocking(p.USART2, p.PA2, p.PA3, uart_cfg).unwrap();
    let (tx, rx) = uart.split();

    // PWM for duty glitch on PA0 (TIM2_CH1) — future: derive via
    // p.PA0 (Peri<PA0>) and TIM3 once a PWM driver is wired in.
    // Note: TIM2 is used by embassy-time; use TIM3 for PWM instead.

    // Shared state between Rx and Tx tasks
    static CAMPAIGN: Signal<CriticalSectionRawMutex, CampaignConfig> = Signal::new();
    static ABORT: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    static GLITCH: Signal<CriticalSectionRawMutex, ()> = Signal::new();

    // Tx task: emits sensor frames according to campaign
    spawner.spawn(tx_task(tx, &CAMPAIGN, &ABORT, &GLITCH).unwrap());

    // Rx task: parses control frames
    spawner.spawn(rx_task(rx, &CAMPAIGN, &ABORT, &GLITCH).unwrap());

    info!("=== Firmware ready ===");
}

/// Transmitter task: runs the campaign, emits faulted frames + ground-truth events.
#[embassy_executor::task]
async fn tx_task(
    mut tx: embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Blocking>,
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
        // Check for new campaign
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

        // Check for abort
        if abort_signal.try_take().is_some() {
            cfg = None;
            info!("Campaign aborted");
        }

        // Check for duty glitch request
        if glitch_signal.try_take().is_some() {
            // Fire one physical glitch on PWM pin
            info!("Duty glitch fired");
        }

        if let Some(c) = cfg.as_mut() {
            if sent >= c.packets {
                let total = c.packets;
                let (frame, n) = encode_end(total, fault_id);
                tx.blocking_write(&frame[..n]).unwrap();
                info!("Campaign completed: {} packets, {} faults", total, fault_id);
                cfg = None;
                continue;
            }

            // Pick fault for this packet
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
                    let (frame, n) = encode_sensor(
                        this_seq,
                        u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]),
                        fid,
                    );
                    tx.blocking_write(&frame[..n]).unwrap();
                    let (frame, n) = encode_fault_event(fid, kind, this_seq);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
                FAULT_DROP => {
                    fault_id += 1;
                    let fid = fault_id;
                    let (frame, n) = encode_fault_event(fid, kind, this_seq);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
                FAULT_DELAY => {
                    fault_id += 1;
                    let fid = fault_id;
                    Timer::after_millis(c.cadence_ms as u64 * 6).await; // 6x cadence = late
                    let (frame, n) = encode_sensor(this_seq, value, 0);
                    tx.blocking_write(&frame[..n]).unwrap();
                    let (frame, n) = encode_fault_event(fid, kind, this_seq);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
                FAULT_REPLAY => {
                    fault_id += 1;
                    let fid = fault_id;
                    let (frame, n) = encode_sensor(this_seq, value, 0);
                    tx.blocking_write(&frame[..n]).unwrap();
                    let (frame, n) = encode_sensor(this_seq, value, 0);
                    tx.blocking_write(&frame[..n]).unwrap();
                    let (frame, n) = encode_fault_event(fid, kind, this_seq);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
                FAULT_BURST => {
                    fault_id += 1;
                    let fid = fault_id;
                    let mut junk = [0u8; 6];
                    for j in junk.iter_mut() {
                        *j = rng.next_u8();
                    }
                    tx.blocking_write(&junk).unwrap();
                    let (frame, n) = encode_sensor(this_seq, value, 0);
                    tx.blocking_write(&frame[..n]).unwrap();
                    let (frame, n) = encode_fault_event(fid, kind, this_seq);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
                FAULT_DUTY => {
                    fault_id += 1;
                    let fid = fault_id;
                    // Physical glitch would go here
                    let (frame, n) = encode_sensor(this_seq, value, 0);
                    tx.blocking_write(&frame[..n]).unwrap();
                    let (frame, n) = encode_fault_event(fid, kind, this_seq);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
                _ => {
                    // Clean
                    let (frame, n) = encode_sensor(this_seq, value, 0);
                    tx.blocking_write(&frame[..n]).unwrap();
                }
            }

            Timer::after_millis(c.cadence_ms as u64).await;
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

/// Receiver task: parses control frames (START, ABORT, PING, GLITCH)
#[embassy_executor::task]
async fn rx_task(
    mut rx: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Blocking>,
    campaign_signal: &'static Signal<CriticalSectionRawMutex, CampaignConfig>,
    abort_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    _glitch_signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    use faultforge_firmware::protocol::{Decoded, FrameParser};
    let mut parser = FrameParser::new();
    let mut byte = [0u8; 1];

    loop {
        if let Ok(()) = rx.blocking_read(&mut byte) {
            if let Some(Decoded::Frame { kind, payload, len }) = parser.push(byte[0]) {
                match kind {
                    K_PING => {
                        let _ = encode_pong();
                        // Need a way to send back - for now just log
                        info!("PING received");
                    }
                    K_START => {
                        if let Some(cfg) = decode_start(&payload[..len]) {
                            campaign_signal.signal(cfg);
                        }
                    }
                    K_ABORT => {
                        abort_signal.signal(());
                    }
                    K_FAULT => {
                        // Ignore ground-truth frames on Rx
                    }
                    _ => {}
                }
            }
        }
        Timer::after_millis(1).await;
    }
}
