use esp_hal::{Async, Blocking};
use esp_hal::gpio::interconnect::{PeripheralInput, PeripheralOutput};
use esp_hal::uart::{Config, DataBits, Instance, Parity, StopBits, Uart};


// Peripheral config (from the Virtual Module) — auto-updated; edit in the module.
pub const BAUDRATE: u32 = 115200;
pub const DATA_BITS: DataBits = DataBits::_8;
pub const PARITY: Parity = Parity::None;
pub const STOP_BITS: StopBits = StopBits::_1;


/// UART1 — blocking driver.
///
/// Generic over the pins so this file never names a GPIO: `main.rs`
/// passes the ones wired on the Pins canvas.
pub fn init<'d>(
    uart: impl Instance + 'd,
    rx: impl PeripheralInput<'d>,
    tx: impl PeripheralOutput<'d>,
) -> Uart<'d, Blocking> {
    let config = Config::default()
        .with_baudrate(BAUDRATE)
        .with_data_bits(DATA_BITS)
        .with_parity(PARITY)
        .with_stop_bits(STOP_BITS);
    // `unwrap`: these values come from the Virtual Module's UI, which
    // range-limits them - a failure here is a bug in the generator,
    // not a runtime condition the firmware could recover from.
    Uart::new(uart, config)
        .unwrap()
        .with_rx(rx)
        .with_tx(tx)
}

/// UART1 — async driver.
///
/// Same construction as `init`, then `.into_async()` — the methods become
/// `*_async` and `.await`-able on the embassy executor.
pub fn init_async<'d>(
    uart: impl Instance + 'd,
    rx: impl PeripheralInput<'d>,
    tx: impl PeripheralOutput<'d>,
) -> Uart<'d, Async> {
    let config = Config::default()
        .with_baudrate(BAUDRATE)
        .with_data_bits(DATA_BITS)
        .with_parity(PARITY)
        .with_stop_bits(STOP_BITS);
    // `unwrap`: these values come from the Virtual Module's UI, which
    // range-limits them - a failure here is a bug in the generator,
    // not a runtime condition the firmware could recover from.
    Uart::new(uart, config)
        .unwrap()
        .with_rx(rx)
        .with_tx(tx)
        .into_async()
}


