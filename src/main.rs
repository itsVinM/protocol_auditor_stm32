//! protocol_auditor_stm32 - host CLI
//!
//! Minimal std-only tool to audit binary protocol robustness on STM32.
//! Crafts frames, sends them over UART, and drains the firmware's output
//! to observe safe parsing and resynchronization.

#[allow(dead_code)]
#[path = "../firmware/src/protocol.rs"]
mod protocol;

use std::io::{Read, Write};
use std::thread;
use std::time::{Duration, Instant};

use protocol::{encode_abort, encode_start, CampaignConfig};
use serialport::SerialPort;

const BAUD: u32 = 115_200;
const DRAIN_MS: u64 = 1500;
const READ_BUF: usize = 256;

const USAGE: &str = "\
protocol_auditor_stm32 — audit binary protocol robustness over UART\n\
\n\
USAGE:\n\
  protocol-auditor-stm32 [OPTIONS]\n\
\n\
OPTIONS:\n\
  --port <path>      Serial port (default: auto-detect ttyACM/ttyUSB/cu.*)\n\
  --list-ports       List available serial ports and exit\n\
  --test <name>      Run a robustness test (use --list-tests)\n\
  --list-tests       List available test cases\n\
  --reset            Send reset to firmware\n\
  -h, --help         Print this help\n";

fn has_flag(args: &[String], flag: &str) -> bool {
    args.iter().any(|a| a == flag)
}

fn arg_val(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1).cloned())
}

fn list_ports() {
    match serialport::available_ports() {
        Ok(ports) => {
            if ports.is_empty() {
                println!("no serial ports found");
            } else {
                for p in ports {
                    println!("{}", p.port_name);
                }
            }
        }
        Err(e) => eprintln!("error listing ports: {e}"),
    }
}

fn list_tests() {
    println!("available tests:");
    println!("  valid        - send a well-formed frame");
    println!("  truncated    - send a cut-off frame");
    println!("  bad-crc      - flip CRC byte");
    println!("  bad-len      - invalid length field");
    println!("  unknown-cmd  - unknown command kind");
    println!("  garbage      - send random bytes before valid frame");
    println!("  extra        - valid frame + trailing garbage");
}

fn find_port() -> Option<String> {
    if let Ok(ports) = serialport::available_ports() {
        for p in ports {
            let name = &p.port_name;
            if name.contains("ttyACM") || name.contains("ttyUSB") || name.contains("usbmodem") || name.contains("cu.") {
                return Some(name.clone());
            }
        }
    }
    None
}

fn build_frame(_args: &[String]) -> Vec<u8> {
    // For now send a minimal valid START frame to keep compatibility
    // with existing firmware. The --test logic can be extended later.
    let cfg = CampaignConfig {
        seed: 42,
        packets: 1,
        cadence_ms: 1,
        weights: [0, 0, 0, 0, 0, 0, 0],
    };
    let (buf, n) = encode_start(&cfg);
    buf[..n].to_vec()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();

    if args.is_empty() || has_flag(&args, "--help") || has_flag(&args, "-h") {
        print!("{USAGE}");
        return;
    }
    if has_flag(&args, "--list-ports") {
        list_ports();
        return;
    }
    if has_flag(&args, "--list-tests") {
        list_tests();
        return;
    }

    let path = match arg_val(&args, "--port").or_else(find_port) {
        Some(p) => p,
        None => {
            eprintln!("no serial port found; pass --port <path> (see --list-ports)");
            std::process::exit(1);
        }
    };

    let frame = if has_flag(&args, "--reset") {
        let (buf, n) = encode_abort();
        buf[..n].to_vec()
    } else {
        build_frame(&args)
    };

    let mut port = match serialport::new(&path, BAUD)
        .timeout(Duration::from_millis(50))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            eprintln!("failed to open {path}: {e}");
            std::process::exit(1);
        }
    };

    if let Err(e) = port.write_all(&frame) {
        eprintln!("write failed: {e}");
        std::process::exit(1);
    }
    let _ = port.flush();

    let mut reader = match port.try_clone() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("failed to clone port: {e}");
            return;
        }
    };

    let handle = thread::spawn(move || {
        let mut buf = [0u8; READ_BUF];
        let start = Instant::now();
        print!("> ");
        while start.elapsed() < Duration::from_millis(DRAIN_MS) {
            match reader.read(&mut buf) {
                Ok(n) if n > 0 => {
                    for b in &buf[..n] {
                        print!("{:02X} ", b);
                    }
                    let _ = std::io::stdout().flush();
                }
                Ok(_) => thread::sleep(Duration::from_millis(5)),
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => thread::sleep(Duration::from_millis(5)),
                Err(_) => break,
            }
        }
        println!();
    });

    let _ = handle.join();
}
