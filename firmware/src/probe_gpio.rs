//! probe::GPIO backend for STM32F401RE - direct registry access

#![allow(dead_code)]

use core::marker::PhantomData;

use protocol_auditor_firmware::probe::GpioOps;

const PIN_TABLE: [(u8, u16, bool); 14] = [
    // (port 0=GPIOA,1=GPIOB, pad, input?, )
    (0, 0, false),  // 0  SCK   PA0 out
    (0, 1, false),  // 1  MOSI  PA1 out
    (0, 2, true),   // 2  MISO  PA2 in
    (0, 3, false),  // 3  CS    PA3 out
    (1, 0, false),  // 4  SCL   PB0 out
    (1, 1, false),  // 5  SDA   PB1 out
    (1, 4, false),  // 6  DQ    PB4 out (open-drain)
    (0, 9, false),  // 7  TX    PA9 out
    (0, 10, true),  // 8  RX    PA10 in
    (1, 5, false),  // 9  (WS)  PB5 out
    (1, 6, false),  // 10 (DHT) PB6 out
    (1, 11, false), // 11 TX_EN PB11 out
    (1, 12, false), // 12 CAN_TX PB12 out
    (1, 13, true),  //13 CAN_RX PB13 in
];

const GPIOA_BASE: u32 = 0x4002_0000;
const RCC_AHB1ENR: u32 = 0x4002_3830;

/// STM32F401RE direct-register GPIO backend for `probe`.
pub struct Stm32Gpio {
    _not_send_sync: PhantomData<*const ()>,
    _dwt_enabled: bool,
}

impl Stm32Gpio {
    pub fn new() -> Self {
        unsafe {
            // Enable GPIOA + GPIOB clocks.
            let rcc = RCC_AHB1ENR as *mut u32;
            rcc.write_volatile(rcc.read_volatile() | (1 << 0) | (1 << 1));

            for &(port, pad, is_input) in &PIN_TABLE {
                let moder = Self::reg(GPIOA_BASE, port, 0x00);
                let m = moder.read_volatile();
                let mode = if is_input { 0u32 } else { 1u32 };
                moder.write_volatile((m & !(3 << (pad * 2))) | (mode << (pad * 2)));
            }
        }

        // DWT_CYCCNT for delay_ns: DEMCR.TRCENA, DWT_CTRL.CYCCNTENA.
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            demcr.write_volatile(demcr.read_volatile() | (1 << 24));
            let ctrl = 0xE000_1000 as *mut u32;
            ctrl.write_volatile(ctrl.read_volatile() | 1);
            (0xE000_1004 as *mut u32).write_volatile(0);
        }

        Self {
            _not_send_sync: PhantomData,
            _dwt_enabled: true,
        }
    }

    #[inline]
    fn reg(base: u32, port: u8, offset: u32) -> *mut u32 {
        (base + (port as u32) * 0x400 + offset) as *mut u32
    }

    #[inline]
    fn now_cycles(&self) -> u32 {
        unsafe { (0xE000_1004 as *const u32).read_volatile() }
    }

    /// Map a logical pin to (port, pad). None if out of range.
    fn pin(&self, pin: u8) -> Option<(u8, u16)> {
        if pin as usize >= PIN_TABLE.len() {
            return None;
        }
        Some((PIN_TABLE[pin as usize].0, PIN_TABLE[pin as usize].1))
    }
}

impl GpioOps for Stm32Gpio {
    fn set(&mut self, pin: u8, val: bool) {
        if let Some((port, pad)) = self.pin(pin) {
            let bsrr = Self::reg(GPIOA_BASE, port, 0x18);
            let b = 1u32 << pad;
            unsafe {
                bsrr.write_volatile(if val { b } else { b << 16 });
            }
        }
    }

    fn get(&mut self, pin: u8) -> bool {
        match self.pin(pin) {
            Some((port, pad)) => {
                let idr = Self::reg(GPIOA_BASE, port, 0x10);
                unsafe { (idr.read_volatile() >> pad) & 1 != 0 }
            }
            None => false,
        }
    }

    fn dir_in(&mut self, pin: u8) {
        if let Some((port, pad)) = self.pin(pin) {
            let moder = Self::reg(GPIOA_BASE, port, 0x00);
            unsafe {
                moder.write_volatile(moder.read_volatile() & !(3 << (pad * 2)));
            }
        }
    }

    fn dir_out(&mut self, pin: u8) {
        if let Some((port, pad)) = self.pin(pin) {
            let moder = Self::reg(GPIOA_BASE, port, 0x00);
            unsafe {
                let m = moder.read_volatile();
                moder.write_volatile((m & !(3 << (pad * 2))) | (1 << (pad * 2)));
            }
        }
    }

    fn delay_ns(&mut self, ns: u64) {
        // DWT cycle counter at 84 MHz -> ~11.9 ns/cycle; ceil to 12 ns.
        let wait = ns.div_ceil(12);
        let start = self.now_cycles();
        loop {
            let cur = self.now_cycles().wrapping_sub(start);
            if (cur as u64) >= wait {
                break;
            }
            core::hint::spin_loop();
        }
    }

    fn log_event(&mut self, _pin: u8, _val: u8, _dir: u8) {
        // Feed-forward: the fault bench triggers on traffic, not on logs.
    }

    fn flush(&mut self) {}
}

impl Default for Stm32Gpio {
    fn default() -> Self {
        Self::new()
    }
}
