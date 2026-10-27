# protocol_auditor_stm32

A minimal host+firmware tool to audit the robustness of a custom length-prefixed binary serial protocol on the STM32F401RE (Nucleo-F401RE).

The goal: verify strict framing, CRC integrity, and safe resynchronization against malformed, truncated, or corrupted frames over UART/USART.

## Features

- **Strict binary framing** – `0x7E` sync byte, `[len][kind][payload][crc8]`
- **CRC-8/ATM validation** – detects corrupted frames
- **Safe resynchronization** – discards garbage bytes and re-acquires sync without crashing
- **Robust input validation** – rejects unknown commands, invalid lengths, and bad CRCs
- **Host-driven tests** – craft frames and observe firmware behavior
- **no_std firmware** – Embassy on STM32F401RE with allocation-free parsing
- **Unit-testable protocol** – `firmware/src/protocol.rs` is shared by source and testable on host

## Architecture

```
Host (Rust, std only)                Firmware (Embassy, no_std)
└─ craft/test frames ──UART──► └─ strict parser + safe frame handler
                        ◄──     └─ minimal feedback
```

## Quick Start

### Host
```sh
cargo run -- --list-ports
cargo run -- --port /dev/ttyACM0
cargo run -- --port /dev/ttyACM0 --reset
cargo run -- --list-tests
```

### Firmware
```sh
cd firmware
cargo build --release --locked --target thumbv7em-none-eabihf
```

Flash with your preferred tool (e.g. `openocd` or copy `.elf`/`.bin` as appropriate).

The wire-protocol unit tests do not need an STM32. They run on the host target:
```sh
cd firmware
cargo test --lib --target "$(rustc -vV | sed -n 's/^host: //p')"
```

## Test Cases (planned)
- `valid` – well-formed frame
- `truncated` – cut-off frame
- `bad-crc` – corrupted CRC
- `bad-len` – invalid length
- `unknown-cmd` – unknown command kind
- `garbage` – random bytes before sync
- `extra` – valid frame + trailing garbage

## Robustness Notes
- Parser never trusts `len` blindly
- Invalid frames are dropped without panicking
- On garbage bytes, receiver hunts for the next `0x7E` sync
- Uses CRC-8/ATM for basic integrity checks

## Layout
- `src/` – host CLI (std only)
- `firmware/src/protocol.rs` – wire format (framing, CRC-8/ATM, encode/decode)
- `firmware/src/main.rs` – no_std Embassy firmware, strict parser
- `firmware/src/lib.rs` – library target for host-side tests
