# HMMD mmWave Sensor — async Rust driver

A `no_std` Rust driver for the [Waveshare HMMD mmWave presence sensor](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor),
implementing its USART command and reporting protocol.

The crate is HAL-agnostic: it only depends on the `embedded-io` /
`embedded-hal` trait families, so it runs on any MCU that has an implementation
for them.

> **Status:** work in progress, not published on crates.io. The API is not
> stable yet, and there are known issues — see [Limitations](#limitations).

## Drivers

Three interchangeable front-ends sit on top of the same protocol core. Pick one:

| Type | I/O model | Traits required | Error handling |
|------|-----------|-----------------|----------------|
| `MicrowaveRadarIOAsync` | `async` | `embedded_io_async::{Read, Write}`, `embedded_hal_async::delay::DelayNs` | `Result<_, RadarError>` |
| `MicrowaveRadarIO` | blocking | `embedded_io::{Read, Write}`, `embedded_hal::delay::DelayNs` | `Result<_, RadarError>` |
| `MicrowaveRadar` | blocking, closure-based | crate-local `UsartTx`, `UsartRx`, `Delay` | `bool` / `Option` |

`MicrowaveRadarIOAsync` is the recommended entry point. `MicrowaveRadar` is the
original closure-based driver, kept for bare-metal setups that have no
`embedded-io` implementation; it swallows transport errors.

All three constructors take their arguments in the order `(tx, rx, delay)`.

## Features

- `no_std`, no allocator, no heap buffers — frames are fixed-size arrays sized
  by const generics.
- Configuration frames: enter config, write/read parameters, save to flash.
- Per-gate trigger and hold thresholds, with dB ↔ raw energy conversion
  (`encode_threshold_value_to_le_bytes` / `decode_threschold_value`).
- Incremental, byte-by-byte `Parser` suitable for interrupt-driven or polled
  UART; it validates header, length, command id and tail, and resets on any
  mismatch.
- Decoders for the two binary output modes:
  - **Report mode** — presence, target distance in cm and 16 per-gate energies
    (`report_normal_mode::HmmdFrame`).
  - **Debug mode** — the 20 × 16 range-Doppler map
    (`report_debug_mode::HmmdRdmapFrame`). See [Limitations](#limitations).

## Hardware

The module exposes a 5-pin header (`J2`):

| Pin | Label | Description                                                          |
|-----|-------|----------------------------------------------------------------------|
| 1   | 3V3   | Power supply, 3.0–3.6 V (typ. 3.3 V)                                 |
| 2   | GND   | Ground                                                               |
| 3   | TX    | Sensor UART transmit, 0–3.3 V                                        |
| 4   | RX    | Sensor UART receive, 0–3.3 V                                         |
| 5   | OT2   | GPIO status output: high = presence, low = absence                   |

Default UART: **115200 baud, 8 data bits, no parity, 1 stop bit**. Data refresh
period is 100 ms.

One **distance gate** is 70 cm and the valid `RangeGate` setting is `0..=15`, so
`RangeGate = N` limits detection to roughly `(N + 1) × 70 cm`.

See the [Waveshare wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor) for
full electrical and protocol details.

### Output modes

The vendor's mode names and this crate's module names do not line up. The
command value sent with `CommandID::ReportMode` (`0x1200`) selects the mode:

| Command value | Waveshare wiki | This crate |
|---------------|----------------|------------|
| `64 00 00 00` | "Normal Mode" — ASCII `ON <gate>` / `OFF` | not implemented |
| `04 00 00 00` | "Report Mode" — 45-byte binary frame | `report_normal_mode`, `SerialCmd::set_report_mode()` |
| `00 00 00 00` | "Debug Mode" — 20 × 16 RDMAP, 1288-byte frame | `report_debug_mode`, `SerialCmd::set_report_debug_mode()` |

## Installation

The crate is not on crates.io yet. Depend on it by git:

```toml
[dependencies]
hmmd_mmwave_sensor_async = { git = "https://github.com/traianistrati-dev/hmmd-mmWave-sensor-lib-async" }
```

Requires **Rust 1.85 or newer** (edition 2024).

> **Note:** at the moment `Cargo.toml` lists `esp-hal` (with the `esp32c3`
> feature) and `embassy-time` as regular dependencies, even though they are only
> used by the examples. Until they are moved to `[dev-dependencies]` or put
> behind a feature flag, the crate only builds for an ESP32-C3 target.

## Usage

### Async (recommended)

```rust,ignore
use hmmd_mmwave_sensor_async::{MicrowaveRadarIOAsync, ParameterID};
// `new_parser()` comes from this trait; it must be in scope.
use hmmd_mmwave_sensor_async::parse_result::{InitParser, decode_threschold_value};
use hmmd_mmwave_sensor_async::report_normal_mode::HmmdFrame;

// esp-hal: `split()` returns `(rx, tx)`, `new()` takes `(tx, rx, delay)`.
let (rx, tx) = uart1.split();
let mut radar = MicrowaveRadarIOAsync::new(tx, rx, embassy_time::Delay);

// Write a few parameters; everything not listed keeps its current value.
radar.set_params_value(&[
    (ParameterID::RangeGate, 1.0),           // gates, 0..=15
    (ParameterID::AbsenseReportDelay, 5.0),  // seconds
]).await;

// Read parameters back. Reads require configuration mode.
let mut parser = HmmdFrame::new_parser();
let mut param_parser = hmmd_mmwave_sensor_async::parameter::ReadParam::new_parser();

radar.begin_config().await?;
let range = radar.get_param_value(ParameterID::RangeGate, &mut param_parser).await?;
let tt_00 = radar
    .get_param_value(ParameterID::TriggerThreshold00, &mut param_parser)
    .await?
    .map(decode_threschold_value); // raw energy -> dB
radar.end_save_config().await?;

// Switch to the 45-byte binary report frame, then stream it.
radar.set_report_mode_35byte_payload().await?;

loop {
    let b = radar.next_byte().await?;
    if parser.feed(b) {
        if let Some(frame) = parser.decode_payload() {
            if frame.present {
                let _distance_cm = frame.distance_cm;
                let _energy_gate_0 = frame.energy[0];
            }
        }
    }
}
```

`parser.feed(byte)` returns `true` once a complete, validated frame has been
assembled; `parser.decode_payload()` then yields the decoded value. The payload
buffer is overwritten by the next frame, so decode before feeding more bytes.

### Blocking

Identical, minus the `.await`, using `MicrowaveRadarIO` and a blocking
`embedded_hal::delay::DelayNs` (for example `esp_hal::delay::Delay`).

### Closure-based

`MicrowaveRadar` takes three closures instead of trait implementations:

```rust,ignore
use hmmd_mmwave_sensor_async::{MicrowaveRadar, ParameterID};
use hmmd_mmwave_sensor_async::parse_result::InitParser;

let delay = |us: u32| { /* busy-wait `us` microseconds */ };
let tx = |bytes: &[u8]| { /* write all bytes to the UART */ };
let rx = || -> Result<u8, nb::Error<()>> { /* read one byte */ Ok(0) };

let mut radar = MicrowaveRadar::new(delay, tx, rx);

let mut param_parser = hmmd_mmwave_sensor_async::parameter::ReadParam::new_parser();
let range: Option<u32> = radar.get_param_value(ParameterID::RangeGate, &mut param_parser);
```

Note that `rx` must return `Result<u8, nb::Error<E>>`, not `Option<u8>`.

## Limitations

- **No timeouts.** Every driver loops until the sensor answers. If the sensor is
  absent or misconfigured, calls block forever. Wrap async calls in
  `embassy_time::with_timeout` (or an equivalent) if that matters.
- **Debug mode does not parse yet.** `report_debug_mode` sets `EXPECTED_CMD_ID`
  to `CommandID::ReportMode`, but an RDMAP frame carries no command-id field, so
  the parser consumes two payload bytes as a command id and resets. It needs to
  be `CommandID::None`.
- **`SerialCmd::delay_ns` is misnamed.** The values (`2000` and `100`) only make
  sense as microseconds; `MicrowaveRadarIO`/`MicrowaveRadarIOAsync` pass them to
  `DelayNs::delay_ns`, which waits ~2 µs — less than the time it takes a single
  byte to arrive at 115200 baud (~87 µs).
- `set_range_delay_with_default_thresholds` and `set_params_value` return `()`
  and discard every error, unlike the rest of the `*IO` API.
- The examples under `src/examples/` are not compiled (the module is commented
  out in `lib.rs`) and are out of date.
- `AbsenseReportDelay` is validated against `1..=99_999_999`, while the vendor
  documents `0..=65535`.

## License

MIT — see [`LICENCE`](LICENCE).