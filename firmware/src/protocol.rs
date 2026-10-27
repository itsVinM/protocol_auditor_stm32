//! The wire contract. The firmware owns this format; the host encodes the two
//! control frames it needs against it.
//!
//! Frame layout (both directions over the same UART):
//!   [0x7E sync] [len] [kind] [payload (len bytes)] [crc8]
//! crc8 covers [len, kind, payload].
//!
//! Guarded by a FrameParser that resyncs on garbage, so corrupted bytes stay
//! observable on the wire instead of desynchronising the stream.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub const SYNC: u8 = 0x7E;

pub const MAX_PAYLOAD: usize = 16;
pub const MAX_FRAME: usize = 3 + MAX_PAYLOAD + 1; // sync + len + kind + payload + crc

// Frame kinds. Host -> injector and injector -> host share one address space.
pub const K_START: u8 = 0x01; // host -> fw: begin a campaign
pub const K_PING: u8 = 0x02; // host -> fw: link handshake
pub const K_ABORT: u8 = 0x03; // host -> fw: stop current campaign

pub const K_SENSOR: u8 = 0x80; // fw -> host: a "measurement" frame (may be faulted)
pub const K_FAULT: u8 = 0x81; // fw -> host: ground-truth injection notice
pub const K_END: u8 = 0x82; // fw -> host: campaign complete
pub const K_PONG: u8 = 0x83; // fw -> host: handshake reply

// Fault kinds carried inside FAULT_EVENT payloads and the campaign weight vector.
pub const FAULT_NONE: u8 = 0;
pub const FAULT_BITFLIP: u8 = 1;
pub const FAULT_BYTE: u8 = 2;
pub const FAULT_DROP: u8 = 3;
pub const FAULT_DELAY: u8 = 4;
pub const FAULT_REPLAY: u8 = 5;
pub const FAULT_BURST: u8 = 6;
pub const FAULT_DUTY: u8 = 7;
pub const N_FAULT_KINDS: usize = 7;

pub const START_PAYLOAD_LEN: usize = 15; // seed4 + packets2 + cadence_ms2 + weights7
pub const SENSOR_PAYLOAD_LEN: usize = 10; // seq2 + value4 + fault_id4
pub const FAULT_PAYLOAD_LEN: usize = 7; // fault_id4 + kind1 + at_seq2
pub const END_PAYLOAD_LEN: usize = 6; // packets2 + faults4

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CampaignConfig {
    pub seed: u32,
    pub packets: u16,
    pub cadence_ms: u16,
    pub weights: [u8; N_FAULT_KINDS],
}

/// Deterministic PRNG shared by the firmware injector and the host simulator so
/// a campaign is reproducible from its seed.
#[derive(Clone, Copy, Debug)]
pub struct Lcg(pub u32);

impl Lcg {
    pub fn new(seed: u32) -> Self {
        // Mix the seed so adjacent seeds diverge immediately.
        Self(seed.wrapping_mul(1664525).wrapping_add(1013904223))
    }

    pub fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1103515245).wrapping_add(12345);
        self.0
    }

    pub fn next_u8(&mut self) -> u8 {
        (self.next_u32() >> 24) as u8
    }

    /// Uniform value in [0, n).
    pub fn below(&mut self, n: u32) -> u32 {
        if n == 0 {
            0
        } else {
            self.next_u32() % n
        }
    }
}

/// CRC-8/ATM (poly 0x07), MSB-first. Initial value 0.
pub fn crc8(data: &[u8]) -> u8 {
    let mut c: u8 = 0;
    for &b in data {
        c ^= b;
        for _ in 0..8 {
            c = if c & 0x80 != 0 {
                (c << 1) ^ 0x07
            } else {
                c << 1
            };
        }
    }
    c
}

/// Encode a frame into a fixed buffer. Returns (buffer, total length incl. crc).
pub fn encode(kind: u8, payload: &[u8]) -> ([u8; MAX_FRAME], usize) {
    let mut buf = [0u8; MAX_FRAME];
    buf[0] = SYNC;
    buf[1] = payload.len() as u8;
    buf[2] = kind;
    buf[3..3 + payload.len()].copy_from_slice(payload);
    let data_end = 3 + payload.len();
    buf[data_end] = crc8(&buf[1..data_end]);
    (buf, data_end + 1)
}

pub fn put_u16(d: &mut [u8], o: usize, v: u16) {
    d[o] = v as u8;
    d[o + 1] = (v >> 8) as u8;
}

pub fn get_u16(d: &[u8], o: usize) -> u16 {
    d[o] as u16 | ((d[o + 1] as u16) << 8)
}

pub fn put_u32(d: &mut [u8], o: usize, v: u32) {
    d[o] = v as u8;
    d[o + 1] = (v >> 8) as u8;
    d[o + 2] = (v >> 16) as u8;
    d[o + 3] = (v >> 24) as u8;
}

pub fn get_u32(d: &[u8], o: usize) -> u32 {
    d[o] as u32 | ((d[o + 1] as u32) << 8) | ((d[o + 2] as u32) << 16) | ((d[o + 3] as u32) << 24)
}

// ---- host -> injector ----

pub fn encode_start(cfg: &CampaignConfig) -> ([u8; MAX_FRAME], usize) {
    let mut p = [0u8; START_PAYLOAD_LEN];
    put_u32(&mut p, 0, cfg.seed);
    put_u16(&mut p, 4, cfg.packets);
    put_u16(&mut p, 6, cfg.cadence_ms);
    p[8..8 + N_FAULT_KINDS].copy_from_slice(&cfg.weights);
    encode(K_START, &p)
}

pub fn encode_abort() -> ([u8; MAX_FRAME], usize) {
    encode(K_ABORT, &[])
}

pub fn decode_start(p: &[u8]) -> Option<CampaignConfig> {
    if p.len() < START_PAYLOAD_LEN {
        return None;
    }
    let mut weights = [0u8; N_FAULT_KINDS];
    weights.copy_from_slice(&p[8..8 + N_FAULT_KINDS]);
    Some(CampaignConfig {
        seed: get_u32(p, 0),
        packets: get_u16(p, 4),
        cadence_ms: get_u16(p, 6),
        weights,
    })
}

// ---- injector -> host ----

pub fn encode_sensor(seq: u16, value: u32, fault_id: u32) -> ([u8; MAX_FRAME], usize) {
    let mut p = [0u8; SENSOR_PAYLOAD_LEN];
    put_u16(&mut p, 0, seq);
    put_u32(&mut p, 2, value);
    put_u32(&mut p, 6, fault_id);
    encode(K_SENSOR, &p)
}

pub fn encode_fault_event(fault_id: u32, kind: u8, at_seq: u16) -> ([u8; MAX_FRAME], usize) {
    let mut p = [0u8; FAULT_PAYLOAD_LEN];
    put_u32(&mut p, 0, fault_id);
    p[4] = kind;
    put_u16(&mut p, 5, at_seq);
    encode(K_FAULT, &p)
}

pub fn encode_end(packets: u16, faults: u32) -> ([u8; MAX_FRAME], usize) {
    let mut p = [0u8; END_PAYLOAD_LEN];
    put_u16(&mut p, 0, packets);
    put_u32(&mut p, 2, faults);
    encode(K_END, &p)
}

pub fn encode_pong() -> ([u8; MAX_FRAME], usize) {
    encode(K_PONG, &[])
}

// ---- streaming parser ----

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Decoded {
    /// A frame passed and its CRC verified.
    Frame {
        kind: u8,
        payload: [u8; MAX_PAYLOAD],
        len: usize,
    },
    /// Enough bytes for a frame arrived but the CRC did not match. The junk is
    /// consumed; this is the observable signature of a corrupted frame.
    BadCrc { kind: u8 },
    /// A run of bytes that were skipped while resynchronising.
    Garbage,
}

pub struct FrameParser {
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_FRAME],
            len: 0,
        }
    }

    /// Feed one byte; returns at most one outcome per call.
    pub fn push(&mut self, b: u8) -> Option<Decoded> {
        if self.len < self.buf.len() {
            self.buf[self.len] = b;
            self.len += 1;
        }
        self.extract()
    }

    fn extract(&mut self) -> Option<Decoded> {
        // Find the first sync byte.
        match self.buf[..self.len].iter().position(|&x| x == SYNC) {
            None => {
                if self.len > 0 {
                    self.len = 0;
                    return Some(Decoded::Garbage);
                }
                return None;
            }
            Some(0) => {}
            Some(i) => {
                self.buf.copy_within(i..self.len, 0);
                self.len -= i;
                // Bytes before the sync are junk.
                if i > 0 {
                    return Some(Decoded::Garbage);
                }
            }
        }

        if self.len < 2 {
            return None; // need len byte
        }
        let plen = self.buf[1] as usize;
        // Guard against absurd lengths (also resyncs on corrupted len bytes).
        if plen > MAX_PAYLOAD {
            // Skip only the SYNC byte and retry; don't consume the next byte as len.
            self.buf.copy_within(1..self.len, 0);
            self.len -= 1;
            return Some(Decoded::Garbage);
        }
        let need = 4 + plen; // sync + len + kind + payload + crc
        if self.len < need {
            return None; // need more bytes
        }

        let kind = self.buf[2];
        let mut full = [0u8; MAX_PAYLOAD];
        full[..plen].copy_from_slice(&self.buf[3..3 + plen]);
        let expected = crc8(&self.buf[1..3 + plen]);
        let got = self.buf[need - 1];

        // Consume exactly the frame.
        self.buf.copy_within(need..self.len, 0);
        self.len -= need;

        if got != expected {
            return Some(Decoded::BadCrc { kind });
        }
        Some(Decoded::Frame {
            kind,
            payload: full,
            len: plen,
        })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    // Debug copy of FrameParser with logging
    struct DebugParser {
        buf: [u8; MAX_FRAME],
        len: usize,
    }

    impl DebugParser {
        fn new() -> Self {
            Self {
                buf: [0; MAX_FRAME],
                len: 0,
            }
        }
        fn push(&mut self, b: u8) -> Option<Decoded> {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
            self.extract()
        }
        fn extract(&mut self) -> Option<Decoded> {
            match self.buf[..self.len].iter().position(|&x| x == SYNC) {
                None => {
                    if self.len > 0 {
                        std::eprintln!("  [DBG] no sync, len={} -> Garbage, clear", self.len);
                        self.len = 0;
                        return Some(Decoded::Garbage);
                    }
                    return None;
                }
                Some(0) => {}
                Some(i) => {
                    std::eprintln!("  [DBG] sync at {}, shift by {}, len={}", i, i, self.len);
                    self.buf.copy_within(i..self.len, 0);
                    self.len -= i;
                    if i > 0 {
                        return Some(Decoded::Garbage);
                    }
                }
            }
            if self.len < 2 {
                std::eprintln!("  [DBG] need len byte, len={}", self.len);
                return None;
            }
            let plen = self.buf[1] as usize;
            std::eprintln!("  [DBG] plen={}, MAX_PAYLOAD={}", plen, MAX_PAYLOAD);
            if plen > MAX_PAYLOAD {
                std::eprintln!("  [DBG] plen > MAX_PAYLOAD, skip 1 byte, len={}", self.len);
                self.buf.copy_within(1..self.len, 0);
                self.len -= 1;
                return Some(Decoded::Garbage);
            }
            let need = 4 + plen; // sync + len + kind + payload + crc
            std::eprintln!("  [DBG] need={}, len={}", need, self.len);
            if self.len < need {
                std::eprintln!("  [DBG] need more bytes");
                return None;
            }
            let kind = self.buf[2];
            let mut full = [0u8; MAX_PAYLOAD];
            full[..plen].copy_from_slice(&self.buf[3..3 + plen]);
            let expected = crc8(&self.buf[1..3 + plen]);
            let got = self.buf[need - 1];
            std::eprintln!(
                "  [DBG] kind={}, expected_crc={:02X}, got_crc={:02X}",
                kind,
                expected,
                got
            );
            self.buf.copy_within(need..self.len, 0);
            self.len -= need;
            if got != expected {
                std::eprintln!("  [DBG] CRC mismatch -> BadCrc");
                return Some(Decoded::BadCrc { kind });
            }
            std::eprintln!("  [DBG] CRC OK -> Frame");
            Some(Decoded::Frame {
                kind,
                payload: full,
                len: plen,
            })
        }
    }

    #[test]
    fn crc8_known_vector() {
        assert_eq!(crc8(b"123456789"), 0xF4);
    }

    #[test]
    fn encode_decode_sensor_roundtrip() {
        let (buf, n) = encode_sensor(0x12AB, 0xDEAD_BEEF, 7);
        assert_eq!(n, 3 + SENSOR_PAYLOAD_LEN + 1);
        assert_eq!(buf[0], SYNC);

        let mut p = DebugParser::new();
        std::eprintln!("buf[..n]: {:?}", &buf[..n]);
        std::eprintln!("n = {}", n);
        std::io::Write::flush(&mut std::io::stderr()).ok();
        let mut frames = Vec::new();
        for &b in &buf[..n] {
            if let Some(d) = p.push(b) {
                std::eprintln!("push byte {:02X} -> {:?}", b, d);
                frames.push(d);
            }
        }
        std::eprintln!("frames: {:?}", frames);
        assert_eq!(frames.len(), 1);
        match frames[0] {
            Decoded::Frame { kind, payload, len } => {
                assert_eq!(kind, K_SENSOR);
                assert_eq!(len, SENSOR_PAYLOAD_LEN);
                assert_eq!(get_u16(&payload, 0), 0x12AB);
                assert_eq!(get_u32(&payload, 2), 0xDEAD_BEEF);
                assert_eq!(get_u32(&payload, 6), 7);
            }
            _ => panic!("expected frame"),
        }
    }

    #[test]
    fn parser_resyncs_through_garbage() {
        let mut p = FrameParser::new();
        let mut out = Vec::new();
        let garbage = [0xA5u8, 0x00, 0xFF, 0x7E, 0x63, 0x7E];
        let (frame, n) = encode_sensor(1, 10, 0);
        for &b in garbage.iter().chain(&frame[..n]) {
            if let Some(d) = p.push(b) {
                out.push(d);
            }
        }
        let has_frame = out
            .iter()
            .any(|d| matches!(d, Decoded::Frame { kind: K_SENSOR, .. }));
        assert!(
            has_frame,
            "parser should find the frame after garbage: {out:?}"
        );
    }

    #[test]
    fn parser_reports_bad_crc() {
        let mut p = FrameParser::new();
        let mut out = Vec::new();
        let (mut frame, n) = encode_sensor(5, 3, 0);
        frame[n - 1] ^= 0xFF;
        for &b in &frame[..n] {
            if let Some(d) = p.push(b) {
                out.push(d);
            }
        }
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0], Decoded::BadCrc { .. }), "got {out:?}");
    }

    #[test]
    fn ping_pong_roundtrip() {
        let (ping, pn) = encode(K_PING, &[]);
        let mut parser = FrameParser::new();
        let mut got = None;
        for &b in &ping[..pn] {
            if let Some(d) = parser.push(b) {
                got = Some(d);
            }
        }
        assert!(matches!(
            got,
            Some(Decoded::Frame {
                kind: K_PING,
                len: 0,
                ..
            })
        ));

        let (pong, n) = encode_pong();
        let mut parser = FrameParser::new();
        let mut got = None;
        for &b in &pong[..n] {
            if let Some(d) = parser.push(b) {
                got = Some(d);
            }
        }
        assert!(matches!(
            got,
            Some(Decoded::Frame {
                kind: K_PONG,
                len: 0,
                ..
            })
        ));
    }

    #[test]
    fn oversized_length_resyncs_to_next_frame() {
        let mut p = FrameParser::new();
        let mut out = Vec::new();
        let (frame, n) = encode_sensor(3, 4, 0);
        let prefix = [SYNC, 0xFF, 0x00];
        for &b in prefix.iter().chain(&frame[..n]) {
            if let Some(d) = p.push(b) {
                out.push(d);
            }
        }
        assert!(
            out.iter()
                .any(|d| matches!(d, Decoded::Frame { kind: K_SENSOR, .. })),
            "oversized length should not consume the next valid frame: {out:?}"
        );
    }

    #[test]
    fn truncated_frame_does_not_emit() {
        let mut p = FrameParser::new();
        let (frame, n) = encode_sensor(1, 2, 3);
        let mut out = Vec::new();
        for &b in &frame[..n - 1] {
            if let Some(d) = p.push(b) {
                out.push(d);
            }
        }
        assert!(
            out.iter().all(|d| !matches!(d, Decoded::Frame { .. })),
            "a truncated frame must wait for more bytes: {out:?}"
        );
    }

    #[test]
    fn start_config_roundtrip() {
        let cfg = CampaignConfig {
            seed: 0xCAFE_1234,
            packets: 1500,
            cadence_ms: 5,
            weights: [0, 20, 0, 30, 10, 5, 2],
        };
        let (buf, n) = encode_start(&cfg);
        let mut p = FrameParser::new();
        let mut decoded = None;
        for &b in &buf[..n] {
            if let Some(Decoded::Frame { payload, len, .. }) = p.push(b) {
                decoded = Some((payload, len));
            }
        }
        let (payload, len) = decoded.expect("start frame");
        let got = decode_start(&payload[..len]).expect("decode start");
        assert_eq!(got.seed, cfg.seed);
        assert_eq!(got.packets, cfg.packets);
        assert_eq!(got.cadence_ms, cfg.cadence_ms);
        assert_eq!(got.weights, cfg.weights);
    }

    #[test]
    fn lcg_deterministic_and_distributed() {
        let mut a = Lcg::new(42);
        let mut b = Lcg::new(42);
        std::eprintln!("initial: a.0={}, b.0={}", a.0, b.0);
        for i in 0..5 {
            std::eprintln!("iter {}: a.0={}, b.0={}", i, a.0, b.0);
            let av = a.next_u32();
            let bv = b.next_u32();
            std::eprintln!("iter {}: av={}, bv={}", i, av, bv);
            assert_eq!(av, bv, "iter {}: a={}, b={}", i, av, bv);
            // Advance both to keep in sync
            let _ = a.below(100);
            let _ = b.below(100);
        }
    }

    #[test]
    fn frame_with_sync_in_payload_parses() {
        let mut p = FrameParser::new();
        let mut out = Vec::new();
        let (f1, n1) = encode(K_SENSOR, &[SYNC, 1, 2, 3]);
        let (f2, n2) = encode(K_SENSOR, &[4, 5, 6, 7]);
        for &b in f1[..n1].iter().chain(&f2[..n2]) {
            if let Some(d) = p.push(b) {
                out.push(d);
            }
        }
        let frames: Vec<_> = out
            .iter()
            .filter_map(|d| match d {
                Decoded::Frame { payload, len, .. } => {
                    let payload = *payload;
                    let l = *len;
                    Some(payload[..l].to_vec())
                }
                _ => None,
            })
            .collect();
        assert_eq!(frames, [&[SYNC, 1, 2, 3][..], &[4, 5, 6, 7][..]]);
    }
}
