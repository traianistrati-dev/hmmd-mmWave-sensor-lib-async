# RUST Driver for HMMD mmWave Sensor

A `no_std` Rust driver for the **HMMD mmWave presence sensor**, implementing its
USART serial protocol. HAL-agnostic: you provide the UART read/write and a delay
function, so it runs on any MCU.


> Hardware and protocol reference:
> [Waveshare HMMD mmWave Sensor Wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor)

## Features

- Pure `no_std`, no allocator required — a single dependency on [`libm`](https://crates.io/crates/libm).
- HAL-independent I/O through three tiny traits (`UsartTx`, `UsartRx`, `Delay`),
  blanket-implemented for closures.
- Build and validate configuration frames: enter config, write/read parameters,
  save to flash.
- Set per-gate trigger/hold thresholds (with dB ↔ raw value encoding).
- Parse both reporting modes:
  - **Normal mode** — presence, target distance (cm) and 16 per-gate energies
    (`HmmdFrame`).
  - **Debug mode** — the 20 × 16 range-Doppler map (`HmmdRdmapFrame`).
- Incremental, byte-by-byte frame `Parser` suitable for interrupt/polled UART.

## Hardware

The module exposes a 5-pin header (`J2`):

| Pin  | Description                    |
|------|--------------------------------|
| 3V3  | Power supply, 3.3 V            |
| GND  | Ground                         |
| TX   | Sensor UART transmit           |
| RX   | Sensor UART receive            |
| OT2  | GPIO output (presence/target)  |

Default UART: **115200 baud, 8N1**. See the
[Waveshare wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor) for full
electrical and protocol details.

## Installation

```toml
[dependencies]
hmmd_mmwave_sensor = "0.1"
```

## Usage

Wire up your platform's UART and a microsecond delay, then drive the sensor:

```rust,ignore
use hmmd_mmwave_sensor::{MicrowaveRadar, data::ParameterID, parameter::ReadParam};

// Platform glue: closures implement the required traits automatically.
let delay_us = |us: u32| { /* busy-wait `us` microseconds */ };
let tx = |bytes: &[u8]| { /* write bytes to the UART */ };
let rx = || -> Option<u8> { /* read one byte if available */ None };

let mut radar = MicrowaveRadar::new(delay_us, tx, rx);

// Configure range + delay and load the default per-gate thresholds.
radar.set_range_delay_with_default_threshold(/* max_range */ 2, /* delay_sec */ 5);

// Read individual parameters.
let mut parser = ReadParam::new_parser();
let range: Option<u32> = radar.get_param_value(ParameterID::RangeGate, &mut parser);
let delay: Option<u32> = radar.get_param_value(ParameterID::AbsenseReportDelay, &mut parser);
```

### Parsing report frames

In normal report mode the sensor streams frames you can decode with `HmmdFrame`:

```rust,ignore
use hmmd_mmwave_sensor::{PayloadDecoder, report_normal_mode::HmmdFrame};

let mut parser = HmmdFrame::new_parser();

// Feed received UART bytes one at a time:
if parser.feed(byte) {
    let frame = HmmdFrame::decode(&parser.payload);
    if frame.present {
        // frame.distance_cm, frame.energy[..]
    }
}
```
