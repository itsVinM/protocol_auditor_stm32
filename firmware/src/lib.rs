//! Testable core of the firmware: the wire contract.
//!
//! The Embassy entry point lives in `main.rs`. Splitting the logic into a
//! library target is what lets `cargo test --lib` run the protocol suite on the
//! host — a `#![no_main]` binary for `thumbv7em-none-eabihf` has no test
//! harness, so logic left in `main.rs` is untestable by construction.

#![no_std]
#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod protocol;
