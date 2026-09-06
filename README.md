diff --git a/README.md b/README.md
index 25616d1..e968d11 100644
--- a/README.md
+++ b/README.md
@@ -1,87 +1,192 @@
-# RUST Driver for HMMD mmWave Sensor
+# HMMD mmWave Sensor — async Rust driver
 
-A `no_std` Rust driver for the **HMMD mmWave presence sensor**, implementing its
-USART serial protocol. HAL-agnostic: you provide the UART read/write and a delay
-function, so it runs on any MCU.
+A `no_std` Rust driver for the [Waveshare HMMD mmWave presence sensor](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor),
+implementing its USART command and reporting protocol.
 
+The crate is HAL-agnostic: it only depends on the `embedded-io` /
+`embedded-hal` trait families, so it runs on any MCU that has an implementation
+for them.
 
-> Hardware and protocol reference:
-> [Waveshare HMMD mmWave Sensor Wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor)
+> **Status:** work in progress, not published on crates.io. The API is not
+> stable yet, and there are known issues — see [Limitations](#limitations).
+
+## Drivers
+
+Three interchangeable front-ends sit on top of the same protocol core. Pick one:
+
+| Type | I/O model | Traits required | Error handling |
+|------|-----------|-----------------|----------------|
+| `MicrowaveRadarIOAsync` | `async` | `embedded_io_async::{Read, Write}`, `embedded_hal_async::delay::DelayNs` | `Result<_, RadarError>` |
+| `MicrowaveRadarIO` | blocking | `embedded_io::{Read, Write}`, `embedded_hal::delay::DelayNs` | `Result<_, RadarError>` |
+| `MicrowaveRadar` | blocking, closure-based | crate-local `UsartTx`, `UsartRx`, `Delay` | `bool` / `Option` |
+
+`MicrowaveRadarIOAsync` is the recommended entry point. `MicrowaveRadar` is the
+original closure-based driver, kept for bare-metal setups that have no
+`embedded-io` implementation; it swallows transport errors.
+
+All three constructors take their arguments in the order `(tx, rx, delay)`.
 
 ## Features
 
-- Pure `no_std`, no allocator required — a single dependency on [`libm`](https://crates.io/crates/libm).
-- HAL-independent I/O through three tiny traits (`UsartTx`, `UsartRx`, `Delay`),
-  blanket-implemented for closures.
-- Build and validate configuration frames: enter config, write/read parameters,
-  save to flash.
-- Set per-gate trigger/hold thresholds (with dB ↔ raw value encoding).
-- Parse both reporting modes:
-  - **Normal mode** — presence, target distance (cm) and 16 per-gate energies
-    (`HmmdFrame`).
-  - **Debug mode** — the 20 × 16 range-Doppler map (`HmmdRdmapFrame`).
-- Incremental, byte-by-byte frame `Parser` suitable for interrupt/polled UART.
+- `no_std`, no allocator, no heap buffers — frames are fixed-size arrays sized
+  by const generics.
+- Configuration frames: enter config, write/read parameters, save to flash.
+- Per-gate trigger and hold thresholds, with dB ↔ raw energy conversion
+  (`encode_threshold_value_to_le_bytes` / `decode_threschold_value`).
+- Incremental, byte-by-byte `Parser` suitable for interrupt-driven or polled
+  UART; it validates header, length, command id and tail, and resets on any
+  mismatch.
+- Decoders for the two binary output modes:
+  - **Report mode** — presence, target distance in cm and 16 per-gate energies
+    (`report_normal_mode::HmmdFrame`).
+  - **Debug mode** — the 20 × 16 range-Doppler map
+    (`report_debug_mode::HmmdRdmapFrame`). See [Limitations](#limitations).
 
 ## Hardware
 
 The module exposes a 5-pin header (`J2`):
 
-| Pin  | Description                    |
-|------|--------------------------------|
-| 3V3  | Power supply, 3.3 V            |
-| GND  | Ground                         |
-| TX   | Sensor UART transmit           |
-| RX   | Sensor UART receive            |
-| OT2  | GPIO output (presence/target)  |
+| Pin | Label | Description                                                          |
+|-----|-------|----------------------------------------------------------------------|
+| 1   | 3V3   | Power supply, 3.0–3.6 V (typ. 3.3 V)                                 |
+| 2   | GND   | Ground                                                               |
+| 3   | TX    | Sensor UART transmit, 0–3.3 V                                        |
+| 4   | RX    | Sensor UART receive, 0–3.3 V                                         |
+| 5   | OT2   | GPIO status output: high = presence, low = absence                   |
+
+Default UART: **115200 baud, 8 data bits, no parity, 1 stop bit**. Data refresh
+period is 100 ms.
 
-Default UART: **115200 baud, 8N1**. See the
-[Waveshare wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor) for full
-electrical and protocol details.
+One **distance gate** is 70 cm and the valid `RangeGate` setting is `0..=15`, so
+`RangeGate = N` limits detection to roughly `(N + 1) × 70 cm`.
+
+See the [Waveshare wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor) for
+full electrical and protocol details.
+
+### Output modes
+
+The vendor's mode names and this crate's module names do not line up. The
+command value sent with `CommandID::ReportMode` (`0x1200`) selects the mode:
+
+| Command value | Waveshare wiki | This crate |
+|---------------|----------------|------------|
+| `64 00 00 00` | "Normal Mode" — ASCII `ON <gate>` / `OFF` | not implemented |
+| `04 00 00 00` | "Report Mode" — 45-byte binary frame | `report_normal_mode`, `SerialCmd::set_report_mode()` |
+| `00 00 00 00` | "Debug Mode" — 20 × 16 RDMAP, 1288-byte frame | `report_debug_mode`, `SerialCmd::set_report_debug_mode()` |
 
 ## Installation
 
+The crate is not on crates.io yet. Depend on it by git:
+
 ```toml
 [dependencies]
-hmmd_mmwave_sensor = "0.1"
+hmmd_mmwave_sensor_async = { git = "https://github.com/traianistrati-dev/hmmd-mmWave-sensor-lib-async" }
 ```
 
+Requires **Rust 1.85 or newer** (edition 2024).
+
+> **Note:** at the moment `Cargo.toml` lists `esp-hal` (with the `esp32c3`
+> feature) and `embassy-time` as regular dependencies, even though they are only
+> used by the examples. Until they are moved to `[dev-dependencies]` or put
+> behind a feature flag, the crate only builds for an ESP32-C3 target.
+
 ## Usage
 
-Wire up your platform's UART and a microsecond delay, then drive the sensor:
+### Async (recommended)
 
 ```rust,ignore
-use hmmd_mmwave_sensor::{MicrowaveRadar, data::ParameterID, parameter::ReadParam};
-
-// Platform glue: closures implement the required traits automatically.
-let delay_ns = |us: u32| { /* busy-wait `us` microseconds */ };
-let tx = |bytes: &[u8]| { /* write bytes to the UART */ };
-let rx = || -> Option<u8> { /* read one byte if available */ None };
+use hmmd_mmwave_sensor_async::{MicrowaveRadarIOAsync, ParameterID};
+// `new_parser()` comes from this trait; it must be in scope.
+use hmmd_mmwave_sensor_async::parse_result::{InitParser, decode_threschold_value};
+use hmmd_mmwave_sensor_async::report_normal_mode::HmmdFrame;
+
+// esp-hal: `split()` returns `(rx, tx)`, `new()` takes `(tx, rx, delay)`.
+let (rx, tx) = uart1.split();
+let mut radar = MicrowaveRadarIOAsync::new(tx, rx, embassy_time::Delay);
+
+// Write a few parameters; everything not listed keeps its current value.
+radar.set_params_value(&[
+    (ParameterID::RangeGate, 1.0),           // gates, 0..=15
+    (ParameterID::AbsenseReportDelay, 5.0),  // seconds
+]).await;
+
+// Read parameters back. Reads require configuration mode.
+let mut parser = HmmdFrame::new_parser();
+let mut param_parser = hmmd_mmwave_sensor_async::parameter::ReadParam::new_parser();
+
+radar.begin_config().await?;
+let range = radar.get_param_value(ParameterID::RangeGate, &mut param_parser).await?;
+let tt_00 = radar
+    .get_param_value(ParameterID::TriggerThreshold00, &mut param_parser)
+    .await?
+    .map(decode_threschold_value); // raw energy -> dB
+radar.end_save_config().await?;
+
+// Switch to the 45-byte binary report frame, then stream it.
+radar.set_report_mode_35byte_payload().await?;
+
+loop {
+    let b = radar.next_byte().await?;
+    if parser.feed(b) {
+        if let Some(frame) = parser.decode_payload() {
+            if frame.present {
+                let _distance_cm = frame.distance_cm;
+                let _energy_gate_0 = frame.energy[0];
+            }
+        }
+    }
+}
+```
 
-let mut radar = MicrowaveRadar::new(delay_ns, tx, rx);
+`parser.feed(byte)` returns `true` once a complete, validated frame has been
+assembled; `parser.decode_payload()` then yields the decoded value. The payload
+buffer is overwritten by the next frame, so decode before feeding more bytes.
 
-// Configure range + delay and load the default per-gate thresholds.
-radar.set_range_delay_with_default_threshold(/* max_range */ 2, /* delay_sec */ 5);
+### Blocking
 
-// Read individual parameters.
-let mut parser = ReadParam::new_parser();
-let range: Option<u32> = radar.get_param_value(ParameterID::RangeGate, &mut parser);
-let delay: Option<u32> = radar.get_param_value(ParameterID::AbsenseReportDelay, &mut parser);
-```
+Identical, minus the `.await`, using `MicrowaveRadarIO` and a blocking
+`embedded_hal::delay::DelayNs` (for example `esp_hal::delay::Delay`).
 
-### Parsing report frames
+### Closure-based
 
-In normal report mode the sensor streams frames you can decode with `HmmdFrame`:
+`MicrowaveRadar` takes three closures instead of trait implementations:
 
 ```rust,ignore
-use hmmd_mmwave_sensor::{PayloadDecoder, report_normal_mode::HmmdFrame};
+use hmmd_mmwave_sensor_async::{MicrowaveRadar, ParameterID};
+use hmmd_mmwave_sensor_async::parse_result::InitParser;
 
-let mut parser = HmmdFrame::new_parser();
+let delay = |us: u32| { /* busy-wait `us` microseconds */ };
+let tx = |bytes: &[u8]| { /* write all bytes to the UART */ };
+let rx = || -> Result<u8, nb::Error<()>> { /* read one byte */ Ok(0) };
 
-// Feed received UART bytes one at a time:
-if parser.feed(byte) {
-    let frame = HmmdFrame::decode(&parser.payload);
-    if frame.present {
-        // frame.distance_cm, frame.energy[..]
-    }
-}
+let mut radar = MicrowaveRadar::new(delay, tx, rx);
+
+let mut param_parser = hmmd_mmwave_sensor_async::parameter::ReadParam::new_parser();
+let range: Option<u32> = radar.get_param_value(ParameterID::RangeGate, &mut param_parser);
 ```
+
+Note that `rx` must return `Result<u8, nb::Error<E>>`, not `Option<u8>`.
+
+## Limitations
+
+- **No timeouts.** Every driver loops until the sensor answers. If the sensor is
+  absent or misconfigured, calls block forever. Wrap async calls in
+  `embassy_time::with_timeout` (or an equivalent) if that matters.
+- **Debug mode does not parse yet.** `report_debug_mode` sets `EXPECTED_CMD_ID`
+  to `CommandID::ReportMode`, but an RDMAP frame carries no command-id field, so
+  the parser consumes two payload bytes as a command id and resets. It needs to
+  be `CommandID::None`.
+- **`SerialCmd::delay_ns` is misnamed.** The values (`2000` and `100`) only make
+  sense as microseconds; `MicrowaveRadarIO`/`MicrowaveRadarIOAsync` pass them to
+  `DelayNs::delay_ns`, which waits ~2 µs — less than the time it takes a single
+  byte to arrive at 115200 baud (~87 µs).
+- `set_range_delay_with_default_thresholds` and `set_params_value` return `()`
+  and discard every error, unlike the rest of the `*IO` API.
+- The examples under `src/examples/` are not compiled (the module is commented
+  out in `lib.rs`) and are out of date.
+- `AbsenseReportDelay` is validated against `1..=99_999_999`, while the vendor
+  documents `0..=65535`.
+
+## License
+
+MIT — see [`LICENCE`](LICENCE).
diff --git a/src/lib.rs b/src/lib.rs
index e6d7594..8a0aaa0 100644
--- a/src/lib.rs
+++ b/src/lib.rs
@@ -9,8 +9,20 @@
 //!
 //! # Overview
 //!
-//! - [`MicrowaveRadar`] is the high-level entry point: it sends commands, waits,
-//!   and validates ACKs or decodes responses.
+//! # Drivers
+//!
+//! The crate ships three interchangeable front-ends over the same protocol core:
+//!
+//! | Type | I/O model | Traits required | Errors |
+//! |------|-----------|-----------------|--------|
+//! | [`MicrowaveRadarIOAsync`] | `async` | [`embedded_io_async::Read`]/[`Write`](embedded_io_async::Write), [`embedded_hal_async::delay::DelayNs`] | `Result<_, RadarError>` |
+//! | [`MicrowaveRadarIO`] | blocking | [`embedded_io::Read`]/[`Write`](embedded_io::Write), [`embedded_hal::delay::DelayNs`] | `Result<_, RadarError>` |
+//! | [`MicrowaveRadar`] | blocking, closure-based | crate-local [`UsartTx`], [`UsartRx`], [`Delay`] | `bool` / `Option` |
+//!
+//! `MicrowaveRadarIOAsync` is the recommended entry point; `MicrowaveRadar` is
+//! the original closure-based driver, kept for bare-metal setups that have no
+//! `embedded-io` implementation.
+//!
 //! - [`SerialCmd`] represents a ready-to-send command frame together with its
 //!   expected ACK payload.
 //! - [`Parser`] is an incremental, byte-by-byte frame parser; concrete frame
@@ -21,17 +33,17 @@
 //! # Example
 //!
 //! ```ignore
-//! let delay_ns = |us: u32| { /* busy-wait `us` microseconds */ };
-//! let tx = |bytes: &[u8]| { /* write bytes to the UART */ };
-//! let rx = || -> Option<u8> { /* read one byte if available */ None };
+//! use hmmd_mmwave_sensor_async::{MicrowaveRadarIOAsync, ParameterID};
+//! // `new_parser()` comes from this trait; it must be in scope.
+//! use hmmd_mmwave_sensor_async::parse_result::InitParser;
 //!
-//! let mut radar = hmmd_mmwave_sensor::MicrowaveRadar::new(delay_ns, tx, rx);
+//! // `tx`/`rx` implement `embedded_io_async::Write`/`Read`,
+//! // `delay` implements `embedded_hal_async::delay::DelayNs`.
+//! let mut radar = MicrowaveRadarIOAsync::new(tx, rx, delay);
 //!
-//! let mut parser = hmmd_mmwave_sensor::parameter::ReadParam::new_parser();
-//! let range = radar.get_param_value(
-//!     hmmd_mmwave_sensor::data::ParameterID::RangeGate,
-//!     &mut parser,
-//! );
+//! let mut parser = hmmd_mmwave_sensor_async::parameter::ReadParam::new_parser();
+//! let range: Option<u32> =
+//!     radar.get_param_value(ParameterID::RangeGate, &mut parser).await?;
 //! ```
 
 #![no_std]
@@ -61,26 +73,30 @@ pub const SEND_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
 /// 4-byte tail terminating every command frame sent to the sensor.
 pub const SEND_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];
 
+/// Factory default UART baud rate of the sensor (8 data bits, no parity,
+/// 1 stop bit).
 pub const BAUD_RATE: u32 = 115200;
 
 
-/// Protocol constants: parameter/command identifiers and frame delimiters.
 /// Identifiers of the sensor's configurable parameters.
 ///
 /// Each variant's value is the 16-bit word placed on the wire (big-endian via
 /// [`get_bytes`](ParameterID::get_bytes)).
 ///
-/// Range is expressed in *gates*: each gate spans ~70 cm, so `RangeGate = N`
-/// configures a maximum detection distance of roughly `N * 70 cm`
-/// (e.g. `RangeGate = 0` ≈ 0–70 cm, `RangeGate = 1` ≈ 0–140 cm). For a given
-/// `RangeGate = N`, the trigger/hold thresholds of gates `00..=N` set the
-/// per-gate detection sensitivity.
+/// Range is expressed in *gates*: one gate is 70 cm and the valid range is
+/// `0..=15`, so `RangeGate = N` configures a maximum detection distance of
+/// roughly `(N + 1) * 70 cm` (`RangeGate = 0` ≈ 0–70 cm, `RangeGate = 1`
+/// ≈ 0–140 cm, `RangeGate = 15` ≈ 0–11.2 m). For a given `RangeGate = N`, the
+/// trigger/hold thresholds of gates `00..=N` set the per-gate sensitivity.
 #[repr(u16)]
 #[derive(Clone, Copy, PartialEq, Eq)]
 pub enum ParameterID{
     /// Maximum detection range, in gates (~70 cm per gate).
     RangeGate = 0x0100,
     /// Absence report delay, in seconds, before a target is reported as gone.
+    ///
+    /// The vendor documents a range of `0..=65535`. Note the typo in the
+    /// variant name (`Absense`); it is kept for backwards compatibility.
     AbsenseReportDelay = 0x0400,
 
     TriggerThreshold00 = 0x1000,
@@ -198,12 +214,20 @@ pub enum CommandID{
     ReadParamAck = 0x0801,
     ReadFirmwareVersion = 0x0000,
     ReadSerialNumber = 0x1100,
-    ReportMode = 0x1200, //64 00 00 00 = Basic (ASCII ON RangeGate 1234 or OFF); 04 00 00 00 = RangeGate with energy; 00 00 00 00 = 20Dopple * 16EnergyGates
+    /// Selects the output mode. The 4-byte command value picks the mode, and
+    /// the vendor's naming differs from this crate's module names:
+    ///
+    /// | Value | Waveshare wiki | This crate |
+    /// |-------|----------------|------------|
+    /// | `64 00 00 00` | "Normal Mode" — ASCII `ON <gate>` / `OFF` | not implemented |
+    /// | `04 00 00 00` | "Report Mode" — 45-byte binary frame | [`report_normal_mode`] / [`SerialCmd::set_report_mode`] |
+    /// | `00 00 00 00` | "Debug Mode" — 20×16 range-Doppler map | [`report_debug_mode`] / [`SerialCmd::set_report_debug_mode`] |
+    ReportMode = 0x1200,
     None = 0xFFFF,
 }
 
 impl CommandID{
-    /// Serializes the command code into its 2 big-endian bytes.f
+    /// Serializes the command code into its 2 big-endian bytes.
     pub fn get_bytes(self) -> [u8; 2]{
         (self as u16).to_be_bytes()
     }
@@ -243,13 +267,16 @@ impl<TxErr, RxErr> From<ReadExactError<RxErr>> for RadarError<TxErr, RxErr> {
 }
 
 
-/// Error produced by any `MicrowaveRadar` operation.
+/// Error produced by any [`MicrowaveRadarIO`] or [`MicrowaveRadarIOAsync`]
+/// operation. (The closure-based [`MicrowaveRadar`] does not use it — it
+/// reports failure as `bool`/`Option`.)
 ///
 /// Written with fully qualified projections on purpose: bounds on type aliases
 /// are not enforced, so `TX: Write` is checked at the use site instead.
 pub type RadarIoError<TX, RX> = RadarError<<TX as embedded_io::ErrorType>::Error, <RX as embedded_io::ErrorType>::Error>;
 
-/// Shorthand for every fallible method of this driver.
+/// Shorthand for every fallible method of [`MicrowaveRadarIO`] and
+/// [`MicrowaveRadarIOAsync`].
 pub type RadarIoResult<T, TX, RX> = Result<T, RadarIoError<TX, RX>>;
 // </Radar Errors>
 
diff --git a/src/parameter.rs b/src/parameter.rs
index 8a77240..b0d3917 100644
--- a/src/parameter.rs
+++ b/src/parameter.rs
@@ -11,6 +11,10 @@ const RESERVED_LEN: usize = 2;
 const HAS_DATA_LENGHT_BYTES: bool = true;
 
 
+/// Decodes a read-parameter ACK payload into the raw 32-bit parameter value.
+///
+/// Threshold parameters come back as raw energy values; convert them with
+/// [`decode_threschold_value`](crate::decode_threschold_value) to get dB.
 pub struct Decoder;
 
 type DecoderType = Decoder;
@@ -26,6 +30,11 @@ impl PayloadDecoder for DecoderType {
 
 
 
+/// Parser factory for read-parameter ACK frames.
+///
+/// Use `ReadParam::new_parser()` with
+/// [`InitParser`](crate::parse_result::InitParser) in scope; the resulting
+/// parser decodes to `u32` and can be reused across successive reads.
 pub struct ReadParam;
 
 
@@ -48,47 +57,39 @@ impl<'a> super::parse_result::InitParser<'a, DecoderType, PAYLOAD_LEN, RESERVED_
 
 
 /// Frame builder for reading a parameter value.
-///
-/// Reading a parameter requires a [`ReadParam`] parser instance to decode the reply.
-///
-/// # Example
-///
-/// ```ignore
-/// let delay_nanos = |ms: u32| {
-///     cortex_m::asm::delay(ms.saturating_mul(&clocks.sysclk().to_Hz() / 1_000_000));
-/// };
-///
-/// let usart1_tx_write_fn = |data: &[u8]| {
-///     for &b in data {
-///         nb::block!(usart1_tx.write(b)).ok();
-///     }
-///     usart1_tx.flush().unwrap_or_default();
-/// };
-///
-/// let usart1_rx_read_fn = || -> Option<u8> {
-///     usart1_rx.read().ok()
-/// };
-///
-/// let mut radar = hmmd_mmwave_sensor::MicrowaveRadar::new(
-///     delay_nanos, usart1_tx_write_fn, usart1_rx_read_fn,
-/// );
-///
-/// let mut parser_params = hmmd_mmwave_sensor::parameter::ReadParam::new_parser();
-///
-/// let range: Option<u32> = radar.get_param_value(
-///     hmmd_mmwave_sensor::data::ParameterID::RangeGate, &mut parser_params);
-/// let delay: Option<u32> = radar.get_param_value(
-///     hmmd_mmwave_sensor::data::ParameterID::AbsenseReportDelay, &mut parser_params);
-/// let tt_00: Option<u32> = radar.get_param_value(
-///     hmmd_mmwave_sensor::data::ParameterID::TriggerThreshold00, &mut parser_params);
-/// let ht_00: Option<u32> = radar.get_param_value(
-///     hmmd_mmwave_sensor::data::ParameterID::HoldThreshold00, &mut parser_params);
-/// ```
-//send FD FC FB FA 04 00 08 00 01 00 04 03 02 01
-//result ACK FD FC FB FA 08 00 08 01 00 00  0F 00 00 00  04 03 02 01
 impl SerialCmd<14, 0>{
 
-
+    /// Builds the "read parameter" command for `param_id`.
+    ///
+    /// The reply must be decoded with a [`ReadParam`] parser, and the sensor
+    /// has to be in configuration mode first (`begin_config`). The returned
+    /// value is raw: thresholds need
+    /// [`decode_threschold_value`](crate::decode_threschold_value) to become dB.
+    ///
+    /// Sent: `FD FC FB FA 04 00 08 00 <param id> 04 03 02 01`.
+    /// ACK:  `FD FC FB FA 08 00 08 01 00 00 <4-byte value> 04 03 02 01`.
+    ///
+    /// # Example
+    ///
+    /// ```ignore
+    /// use hmmd_mmwave_sensor_async::{MicrowaveRadarIOAsync, ParameterID};
+    /// use hmmd_mmwave_sensor_async::parse_result::{InitParser, decode_threschold_value};
+    ///
+    /// let mut radar = MicrowaveRadarIOAsync::new(tx, rx, embassy_time::Delay);
+    /// let mut parser = hmmd_mmwave_sensor_async::parameter::ReadParam::new_parser();
+    ///
+    /// radar.begin_config().await?;
+    ///
+    /// let range = radar.get_param_value(ParameterID::RangeGate, &mut parser).await?;
+    /// let delay = radar.get_param_value(ParameterID::AbsenseReportDelay, &mut parser).await?;
+    ///
+    /// let tt_00 = radar
+    ///     .get_param_value(ParameterID::TriggerThreshold00, &mut parser)
+    ///     .await?
+    ///     .map(decode_threschold_value); // dB
+    ///
+    /// radar.end_save_config().await?;
+    /// ```
     pub fn read_param_value(param_id: ParameterID) -> Self{
 
         let cmd_id_2b = CommandID::ReadParam.get_bytes();
@@ -111,8 +112,6 @@ impl SerialCmd<14, 0>{
 
 
 /// Frame builder for writing a parameter value.
-//send FD FC FB FA 08 00 07 00 01 00 02 00 00 00 04 03 02 01
-//result ACK FD FC FB FA_ 04 00 _07 01_ 00 00 04 03 02 01
 impl SerialCmd<18, 4>{
 
 
@@ -122,10 +121,16 @@ impl SerialCmd<18, 4>{
     /// written as a raw integer; trigger/hold thresholds are encoded from dB via
     /// [`encode_threshold_value_to_le_bytes`](crate::encode_threshold_value_to_le_bytes).
     ///
+    /// Sent: `FD FC FB FA 08 00 07 00 <param id> <4-byte value> 04 03 02 01`.
+    /// ACK:  `FD FC FB FA 04 00 07 01 00 00 04 03 02 01`.
+    ///
     /// Each value is range-checked and falls back to
     /// [`default_value`](ParameterID::default_value) when out of range:
-    /// `RangeGate` must be `< 16`, `AbsenseReportDelay` in `1..=99_999_999`, and
-    /// thresholds in `1.0..=90.0` dB.
+    /// `RangeGate` must be `< 16`, `AbsenseReportDelay` in `1..=99_999_999`,
+    /// and thresholds in `1.0..=90.0` dB.
+    ///
+    /// Note that the vendor documents `AbsenseReportDelay` as `0..=65535`; the
+    /// wider bound accepted here is not validated by the sensor.
     pub fn set_param_value(param_id: ParameterID, param_value: f32) -> Self{
 
         let param_value_4b = match &param_id {
diff --git a/src/parse_result.rs b/src/parse_result.rs
index 379b181..5544190 100644
--- a/src/parse_result.rs
+++ b/src/parse_result.rs
@@ -13,8 +13,17 @@ enum State {
 
 
 
+/// Turns a validated frame payload into a typed value.
+///
+/// Each frame kind supplies its own decoder — see
+/// [`report_normal_mode::Decoder`](crate::report_normal_mode::Decoder),
+/// [`report_debug_mode::Decoder`](crate::report_debug_mode::Decoder) and
+/// [`parameter::Decoder`](crate::parameter::Decoder). The unit type `()` is
+/// implemented as a no-op decoder for parsers used only to match ACK bytes.
 pub trait PayloadDecoder {
+    /// The value produced from a complete payload.
     type Output;
+    /// Decodes `payload`, which is always exactly `PAYLOAD_LEN` bytes long.
     fn decode(&self, payload: &[u8]) -> Self::Output;
 }
 
@@ -26,6 +35,17 @@ impl PayloadDecoder for () {
 }
 
 
+/// Builds a [`Parser`] pre-configured for one frame kind.
+///
+/// Implemented by [`HmmdFrame`](crate::report_normal_mode::HmmdFrame),
+/// [`HmmdRdmapFrame`](crate::HmmdRdmapFrame) and
+/// [`ReadParam`](crate::parameter::ReadParam). **This trait must be in scope**
+/// for `Type::new_parser()` to resolve:
+///
+/// ```ignore
+/// use hmmd_mmwave_sensor_async::parse_result::InitParser;
+/// let mut parser = hmmd_mmwave_sensor_async::report_normal_mode::HmmdFrame::new_parser();
+/// ```
 pub trait InitParser
 <'a,
 DECODER: PayloadDecoder,
@@ -45,6 +65,15 @@ const HAS_DATA_LENGHT_BYTES: bool
 /// complete, valid frame has been assembled, at which point
 /// [`payload`](Parser::payload) holds the useful bytes. Header/length/command-id/
 /// tail are validated along the way and any mismatch resets the machine.
+///
+/// The const generics describe the frame layout: `PAYLOAD_LEN` and
+/// `RESERVED_LEN` are byte counts, `EXPECTED_CMD_ID` is the command id the
+/// frame must carry (use [`CommandID::None`](crate::CommandID::None) for frames
+/// that have no command-id field), and `HAS_DATA_LENGHT_BYTES` says whether a
+/// 2-byte little-endian length field follows the header.
+///
+/// Note the typo in `HAS_DATA_LENGHT_BYTES` (should be `LENGTH`); it is part of
+/// the public API and kept for backwards compatibility.
 pub struct Parser
 <'a,
 DECODER,
@@ -78,6 +107,12 @@ const HAS_DATA_LENGHT_BYTES: bool
 Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>
 where DECODER: PayloadDecoder
 {
+    /// Creates a parser for frames delimited by `header` and `tail`.
+    ///
+    /// Pass `None` for `result_decoder` when the parser is only used to match
+    /// raw ACK bytes; [`decode_payload`](Self::decode_payload) then always
+    /// returns `None`. Prefer the ready-made constructors from
+    /// [`InitParser`].
     pub const fn new(header: &'a[u8; 4], tail: &'a [u8; 4], result_decoder: Option<DECODER>) -> Self {
         Self {
             state: State::Header(0),
@@ -93,6 +128,12 @@ where DECODER: PayloadDecoder
 
 
 
+    /// Decodes the payload captured by the last successful
+    /// [`feed`](Self::feed), or `None` if the parser was built without a
+    /// decoder.
+    ///
+    /// Only meaningful right after `feed` returned `true`; the payload buffer
+    /// is overwritten by the next frame.
     pub fn decode_payload(&self) -> Option<DECODER::Output> {
         if let Some(decoder) = &self.result_decoder {
             return Some(decoder.decode(&self.payload));
@@ -104,6 +145,9 @@ where DECODER: PayloadDecoder
     const fn has_cmd_id() -> bool {
         EXPECTED_CMD_ID != super::CommandID::None.as_u16()
     }
+    /// Discards any partially received frame and restarts at the header.
+    ///
+    /// The payload buffer is left untouched.
     pub fn clear(&mut self) {
         self.reset();
     }
@@ -254,6 +298,12 @@ where DECODER: PayloadDecoder
 }
 
 /// Decodes a raw threshold value back into dB (`10 * log10(value)`).
+///
+/// Inverse of
+/// [`encode_threshold_value_to_le_bytes`](crate::encode_threshold_value_to_le_bytes).
+/// Returns `0.0` for an input of `0`.
+///
+/// Note the typo in the name (`threschold`); it is part of the public API.
 pub fn decode_threschold_value(value: u32) -> f32 {
     if value == 0 {
         return 0.0;
diff --git a/src/radar_fn.rs b/src/radar_fn.rs
index 8703078..2d8c05a 100644
--- a/src/radar_fn.rs
+++ b/src/radar_fn.rs
@@ -1,11 +1,21 @@
-//! High-level driver ([`MicrowaveRadar`]) and the HAL-agnostic I/O traits.
+//! Closure-based driver ([`MicrowaveRadar`]) and its crate-local I/O traits.
+//!
+//! This is the original front-end: it takes three closures instead of
+//! `embedded-io` implementations. It is fully blocking and swallows I/O errors
+//! (`bool` / `Option` instead of `Result`). Prefer
+//! [`MicrowaveRadarIOAsync`](crate::MicrowaveRadarIOAsync) or
+//! [`MicrowaveRadarIO`](crate::MicrowaveRadarIO) in new code.
 
 use super::{ParameterID, SerialCmd, PayloadDecoder};
 
 /// High-level driver for the HMMD mmWave sensor.
 ///
 /// Generic over the three I/O traits ([`Delay`], [`UsartTx`], [`UsartRx`]),
-/// so it stays independent of any concrete HAL. Construct it with (MicrowaveRadar::new).
+/// so it stays independent of any concrete HAL. Construct it with
+/// [`MicrowaveRadar::new`].
+///
+/// **Blocking:** every method busy-waits until the sensor answers. There is no
+/// timeout, so a disconnected sensor will hang the caller.
 pub struct MicrowaveRadar<DELAY: Delay, TX: UsartTx, RX: UsartRx>{
 
     delay: DELAY,
@@ -32,7 +42,8 @@ impl<F> UsartTx for F where F: FnMut(&[u8]),
 
 /// Receives a single byte from the sensor's USART link.
 ///
-/// Blanket-implemented for any `FnMut() -> Result<u8, nb::Error>.
+/// Blanket-implemented for any `FnMut() -> Result<u8, nb::Error<E>>`, so a
+/// closure wrapping a `nb`-style HAL read can be passed directly.
 pub trait UsartRx {
     type Error;
     fn read_byte(&mut self) -> Result<u8, nb::Error<Self::Error>>;
@@ -48,8 +59,18 @@ impl<F, E> UsartRx for F where F: FnMut() -> Result<u8, nb::Error<E>>,
 
 /// Blocking delay used to pace command/response exchanges.
 ///
-/// Blanket-implemented for any `Fn(u32)`. The unit of the argument is defined by
-/// the implementation (the crate passes the value of `SerialCmd::delay_us`)
+/// Blanket-implemented for any `Fn(u32)`.
+///
+/// # Units
+///
+/// The crate passes [`SerialCmd::delay_ns`](crate::SerialCmd) straight into
+/// this method, and the two names disagree: the field is called `delay_ns`
+/// while the method is called `delay_us`. Your closure decides what the number
+/// means. The values built into the command builders are `2000` (config
+/// commands) and `100` (report-mode commands); those only make sense as
+/// **microseconds or larger** — at 115200 baud a single byte already takes
+/// ~87 µs, so treating them as nanoseconds leaves no time for the ACK.
+/// Interpret the argument as microseconds.
 pub trait Delay {
     fn delay_us(&self, ms: u32);
 }
@@ -69,14 +90,20 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
         Self { delay: delay_fn, tx, rx}
     }
 
-    /// Configures the maximum range and delay, loads the default trigger/hold
-    /// thresholds for all 16 distance gates, saves the configuration, and puts
-    /// the sensor into report mode.
+    /// Configures the maximum range and the absence delay, then writes the
+    /// factory-default trigger/hold thresholds for all 16 distance gates and
+    /// saves the configuration.
     ///
-    /// All parameters are written between `begin_config` and `end_save_config`.
+    /// `max_range` is a gate index (`0..=15`, 70 cm per gate) and `delay_sec`
+    /// is the absence report delay in seconds. All writes happen between
+    /// [`begin_config`](Self::begin_config) and
+    /// [`end_save_config`](Self::end_save_config); every parameter must be
+    /// written before `end_save_config()` for the configuration to reach the
+    /// sensor's flash.
     ///
-    /// **Note:** every parameter must be written before `end_save_config()` for
-    /// the configuration to be persisted to the sensor's flash.
+    /// This does **not** switch the sensor into a report mode — call
+    /// [`set_report_mode_35byte_payload`](Self::set_report_mode_35byte_payload)
+    /// afterwards. All ACK failures are ignored.
     pub fn set_range_delay_with_default_thresholds(&mut self, max_range: u8, delay_sec: u32){
 
         if (self.begin_config() && self.begin_config()) {
@@ -127,6 +154,13 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
 
 
 
+    /// Writes a batch of parameters, keeping every parameter not listed in
+    /// `params_values` at the value currently stored in the sensor (falling
+    /// back to [`ParameterID::default_value`] if it cannot be read).
+    ///
+    /// Thresholds are given in dB; `RangeGate` and `AbsenseReportDelay` as raw
+    /// values. All writes are wrapped in `begin_config` / `end_save_config`.
+    /// ACK failures are ignored.
     pub fn set_params_value(&mut self, params_values: &[(ParameterID, f32)]){
 
         use super::parse_result::InitParser;
@@ -246,7 +280,10 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
 
 
 
-    /// Reads a single byte, if available, and hands it to `read_fn`.
+    /// Reads one byte and hands it to `read_fn`.
+    ///
+    /// **Blocks** until a byte arrives: the underlying loop retries on
+    /// `nb::WouldBlock` *and* on transport errors, so it never gives up.
     pub fn read_byte(&mut self, mut read_fn: impl FnMut(u8)){
 
         if let Some(b) = self.next_byte() {
@@ -265,7 +302,8 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
         }
     }
 
-    /// Blocks using the configured delay function for `ms` time units.
+    /// Blocks using the configured [`Delay`] closure. See [`Delay`] for the
+    /// unit caveat — the argument is passed through unchanged.
     pub fn delay_us(&self, ms: u32) {
 
         self.delay.delay_us(ms);
@@ -273,8 +311,16 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
 
     /// Reads the value of a single parameter from the sensor.
     ///
-    /// `parser` must be created with (crate::parameter::ReadParam::new_parser).
-    /// Returns `None` on timeout or an invalid reply.
+    /// `parser` must be created with
+    /// [`ReadParam::new_parser()`](crate::parameter::ReadParam), which requires
+    /// [`InitParser`](crate::parse_result::InitParser) to be in scope. The
+    /// sensor must already be in configuration mode
+    /// ([`begin_config`](Self::begin_config)).
+    ///
+    /// Returns the raw 4-byte value as `u32`. Threshold parameters must be run
+    /// through [`decode_threschold_value`](crate::decode_threschold_value) to
+    /// get dB. Returns `None` only if the parser carries no decoder; there is
+    /// no timeout, so an unresponsive sensor blocks forever.
     // pub fn get_param_value<const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16,const HAS_DATA_LENGHT_BYTES: bool>(
     // &mut self
     // ,param_id:ParameterID
@@ -295,10 +341,11 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
 
     }
 
-    /// Sends a command, then parses the reply with `parser` and decodes it with `decode`.
+    /// Sends a command, then feeds received bytes to `parser` until a complete
+    /// frame is assembled, and returns the decoded payload.
     ///
-    /// Returns the decoded result, or `None` if no valid frame arrives before the
-    /// internal idle timeout.
+    /// Returns `None` only if the parser was built without a decoder. There is
+    /// **no timeout**: if the sensor never answers, this never returns.
     pub fn send_cmd_and_get_result<DECODER, const S: usize, const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16, const HAS_DATA_LENGHT_BYTES: bool>(
         &mut self,
         data: SerialCmd<S, 0>,
@@ -325,8 +372,10 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
     /// Sends a command and checks whether the received ACK matches the expected
     /// payload.
     ///
-    /// Returns `true` on a match — or immediately when the command defines no ACK
-    /// payload (`result_payload_ack` empty); returns `false` on mismatch or timeout.
+    /// Returns `true` on a match — or immediately when the command defines no
+    /// ACK payload (`result_payload_ack` empty); returns `false` on mismatch.
+    /// There is **no timeout**: an unresponsive sensor blocks forever rather
+    /// than returning `false`.
     pub fn send_cmd_and_check_ack_result< const S: usize, const R: usize>(&mut self, data: SerialCmd<S, R>) -> bool{
 
         self.tx.write_bytes(&data.send);
@@ -356,6 +405,10 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
 
 
     /// Enters configuration mode (the "enable config" command).
+    ///
+    /// Must succeed before any parameter read or write. Callers in this crate
+    /// send it twice, as the sensor frequently ignores the first attempt after
+    /// power-up.
     pub fn begin_config(&mut self) -> bool{
         self.send_cmd_and_check_ack_result(SerialCmd::begin_config())
     }
@@ -365,10 +418,18 @@ impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
         self.send_cmd_and_check_ack_result(SerialCmd::end_save_config())
     }
 
+    /// Switches the sensor to the 45-byte binary report frame (35-byte
+    /// payload) decoded by [`HmmdFrame`](crate::report_normal_mode::HmmdFrame).
+    ///
+    /// This is Waveshare's *Report Mode* (command value `04 00 00 00`). The
+    /// command has no ACK, so nothing is verified.
     pub fn set_report_mode_35byte_payload(&mut self){
         self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode());
     }
 
+    /// Switches the sensor to debug mode: 1288-byte frames carrying the
+    /// 1280-byte range-Doppler map decoded by
+    /// [`HmmdRdmapFrame`](crate::HmmdRdmapFrame). The command has no ACK.
     pub fn set_report_debug_mode_1280byte_payload(&mut self){
         self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode());
     }
diff --git a/src/radar_io.rs b/src/radar_io.rs
index 7cf6020..cf74c2c 100644
--- a/src/radar_io.rs
+++ b/src/radar_io.rs
@@ -1,12 +1,17 @@
-//! `MicrowaveRadar` built on the standard blocking `embedded-io` /
-//! `embedded-hal` traits — no closures, and no custom
+//! [`MicrowaveRadarIO`] — blocking driver built on the standard `embedded-io` /
+//! `embedded-hal` traits, with no closures and none of the crate-local
 //! `UsartTx` / `UsartRx` / `Delay` traits.
 //!
-//! Mirror of `radar_.rs`: same API, same error type, no ``/``.
+//! Mirror of [`radar_fn`](crate::radar_fn): same command set, but errors are
+//! surfaced as [`RadarError`] instead of being collapsed into `bool`/`Option`,
+//! and there is no dependency on `nb`.
 //!
-//! Cargo.toml:
-//!   embedded-io  = "0.6"
-//!   embedded-hal = "1.0"
+//! Required dependencies (as declared in this crate's `Cargo.toml`):
+//!
+//! ```toml
+//! embedded-io  = "0.7"
+//! embedded-hal = "1.0"
+//! ```
 
 use embedded_hal::delay::DelayNs;
 use embedded_io::{ Read, Write};
@@ -16,11 +21,12 @@ use super::{ParameterID, PayloadDecoder, SerialCmd, RadarError, RadarIoResult};
 
 /// Driver generic over any TX/RX pair implementing the standard blocking I/O
 /// traits. On esp-hal: `let (rx, tx) = uart.split();` — both halves implement
-/// `Read`/`Write` when the driver is in `Blocking` mode.
+/// `Read`/`Write` when the driver is in `Blocking` mode. Mind the order:
+/// `split()` yields `(rx, tx)` while [`new`](Self::new) takes `(tx, rx, delay)`.
 ///
 /// Note: every call here busy-waits until the exchange completes. Under an
-///  runtime this stalls the whole executor — use `MicrowaveRadarAsync`
-/// there instead.
+/// async runtime this stalls the whole executor — use
+/// [`MicrowaveRadarIOAsync`](crate::MicrowaveRadarIOAsync) there instead.
 pub struct MicrowaveRadarIO<TX, RX, DELAY> {
     tx: TX,
     rx: RX,
@@ -33,9 +39,11 @@ TX: Write,
 RX: Read,
 DELAY: DelayNs,
 {
-    /// Delays supported as a parameter:
-    /// `esp_hal::delay::Delay`
-    /// any `embedded_hal::delay::DelayNs`
+    /// Builds the driver from the TX half, the RX half and a delay provider,
+    /// **in that order**.
+    ///
+    /// Accepted delay implementations include `esp_hal::delay::Delay` and any
+    /// other [`embedded_hal::delay::DelayNs`].
     #[allow(dead_code)]
     pub fn new(tx: TX, rx: RX, delay: DELAY) -> Self {
         Self { tx, rx, delay }
@@ -125,19 +133,36 @@ DELAY: DelayNs,
         self.send_cmd_and_check_ack_result(SerialCmd::end_save_config())
     }
 
-    /// High-level helpers propagate the error with `?` instead of swallowing
+    /// Switches the sensor to the 45-byte binary report frame (35-byte
+    /// payload) decoded by [`HmmdFrame`](crate::report_normal_mode::HmmdFrame).
+    ///
+    /// This is Waveshare's *Report Mode* (command value `04 00 00 00`). The
+    /// command defines no ACK payload, so `Ok(())` only means the frame was
+    /// written to the UART.
     #[allow(dead_code)]
     pub fn set_report_mode_35byte_payload(&mut self) -> RadarIoResult<(), TX, RX> {
         self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode())
     }
 
+    /// Switches the sensor to debug mode: 1288-byte frames carrying the
+    /// 1280-byte range-Doppler map decoded by
+    /// [`HmmdRdmapFrame`](crate::HmmdRdmapFrame). No ACK is defined.
     #[allow(dead_code)]
     pub fn set_report_debug_mode_1280byte_payload(&mut self) -> RadarIoResult<(), TX, RX>{
         self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode())
     }
 
     /// Reads a single parameter from the sensor.
-    /// `parser` must be created with `crate::parameter::ReadParam::new_parser()`.
+    ///
+    /// `parser` must be created with
+    /// [`ReadParam::new_parser()`](crate::parameter::ReadParam), which requires
+    /// [`InitParser`](crate::parse_result::InitParser) to be in scope, and the
+    /// sensor must already be in configuration mode
+    /// ([`begin_config`](Self::begin_config)).
+    ///
+    /// The value is returned raw; threshold parameters must be converted with
+    /// [`decode_threschold_value`](crate::decode_threschold_value) to obtain dB.
+    /// `Ok(None)` means the parser carried no decoder.
     pub fn get_param_value<
     'a,
     DECODER,
@@ -156,14 +181,18 @@ DELAY: DelayNs,
         self.send_cmd_and_get_result(SerialCmd::read_param_value(param_id), parser)
     }
 
-    /// Configures the maximum range and delay, loads the default trigger/hold
-    /// thresholds for all 16 distance gates, saves the configuration, and puts
-    /// the sensor into report mode.
+    /// Configures the maximum range and the absence delay, then writes the
+    /// factory-default trigger/hold thresholds for all 16 distance gates and
+    /// saves the configuration.
     ///
-    /// All parameters are written between `begin_config` and `end_save_config`.
+    /// `max_range` is a gate index (`0..=15`, 70 cm per gate); `delay_sec` is
+    /// the absence report delay in seconds. Every parameter must be written
+    /// before `end_save_config()` for the configuration to reach flash.
     ///
-    /// **Note:** every parameter must be written before `end_save_config()` for
-    /// the configuration to be persisted to the sensor's flash.
+    /// This does **not** switch the sensor into a report mode — call
+    /// [`set_report_mode_35byte_payload`](Self::set_report_mode_35byte_payload)
+    /// afterwards. Unlike the rest of this type, the method returns `()` and
+    /// discards every error.
     #[allow(dead_code)]
     pub fn set_range_delay_with_default_thresholds(&mut self, max_range: u8, delay_sec: u32){
 
@@ -212,6 +241,12 @@ DELAY: DelayNs,
         // self.set_report_mode_35byte_payload();
 
     }
+    /// Writes a batch of parameters, keeping every parameter not listed in
+    /// `params_values` at the value currently stored in the sensor (falling
+    /// back to [`ParameterID::default_value`] if it cannot be read).
+    ///
+    /// Thresholds are given in dB; `RangeGate` and `AbsenseReportDelay` as raw
+    /// values. Returns `()` and discards every error.
     #[allow(dead_code)]
     pub fn set_params_value(&mut self, params_values: &[(ParameterID, f32)]){
 
diff --git a/src/radar_io_async.rs b/src/radar_io_async.rs
index 32af365..f798e89 100644
--- a/src/radar_io_async.rs
+++ b/src/radar_io_async.rs
@@ -1,10 +1,16 @@
-//! MicrowaveRadarAsync built on the standard `embedded-io-async` /
-//! `embedded-hal-async` traits — no closures, and no custom
-//! `UsartTx` / `UsartRx` / `Delay` traits.
+//! [`MicrowaveRadarIOAsync`] — the crate's primary driver, built on the
+//! standard `embedded-io-async` / `embedded-hal-async` traits, with no closures
+//! and none of the crate-local `UsartTx` / `UsartRx` / `Delay` traits.
 //!
-//! Cargo.toml:
-//!   embedded-io-async  = "0.6"
-//!   embedded-hal-async = "1.0"
+//! Every method awaits instead of busy-waiting, so the executor stays free
+//! while the sensor answers. Errors are surfaced as [`RadarError`].
+//!
+//! Required dependencies (as declared in this crate's `Cargo.toml`):
+//!
+//! ```toml
+//! embedded-io-async  = "0.7"
+//! embedded-hal-async = "1.0"
+//! ```
 
 use embedded_hal_async::delay::DelayNs;
 use embedded_io_async::{Read, Write};
@@ -14,7 +20,11 @@ use super::{ParameterID, PayloadDecoder, SerialCmd, RadarError, RadarIoResult};
 
 /// Driver generic over any TX/RX pair implementing the standard async I/O
 /// traits. On esp-hal: `let (rx, tx) = uart.split();` — both halves implement
-/// `Read`/`Write` when the driver is in `Async` mode.
+/// `Read`/`Write` when the driver is in `Async` mode. Mind the order:
+/// `split()` yields `(rx, tx)` while [`new`](Self::new) takes `(tx, rx, delay)`.
+///
+/// None of the methods has a timeout; wrap calls in `embassy_time::with_timeout`
+/// (or an equivalent) if the sensor may be absent.
 pub struct MicrowaveRadarIOAsync<TX, RX, DELAY> {
     tx: TX,
     rx: RX,
@@ -27,9 +37,11 @@ TX: Write,
 RX: Read,
 DELAY: DelayNs,
 {
-    ///delay suported as parameter:
-    /// embassy_time::Delay
-    /// embedded_hal_async::delay::DelayNs
+    /// Builds the driver from the TX half, the RX half and a delay provider,
+    /// **in that order**.
+    ///
+    /// Accepted delay implementations include `embassy_time::Delay` and any
+    /// other [`embedded_hal_async::delay::DelayNs`].
     pub fn new(tx: TX, rx: RX, delay: DELAY) -> Self {
         Self { tx, rx, delay }
     }
@@ -123,20 +135,36 @@ DELAY: DelayNs,
         self.send_cmd_and_check_ack_result(SerialCmd::end_save_config()).await
     }
 
-    /// High-level helpers are async too, and propagate the error with `?`
-    /// instead of swallowing it into a `bool`.
+    /// Switches the sensor to the 45-byte binary report frame (35-byte
+    /// payload) decoded by [`HmmdFrame`](crate::report_normal_mode::HmmdFrame).
+    ///
+    /// This is Waveshare's *Report Mode* (command value `04 00 00 00`). The
+    /// command defines no ACK payload, so `Ok(())` only means the frame was
+    /// written to the UART.
     #[allow(dead_code)]
     pub async fn set_report_mode_35byte_payload(&mut self) -> RadarIoResult<(), TX, RX>{
         self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode()).await
     }
 
+    /// Switches the sensor to debug mode: 1288-byte frames carrying the
+    /// 1280-byte range-Doppler map decoded by
+    /// [`HmmdRdmapFrame`](crate::HmmdRdmapFrame). No ACK is defined.
     #[allow(dead_code)]
     pub async fn set_report_debug_mode_1280byte_payload(&mut self) -> RadarIoResult<(), TX, RX>{
         self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode()).await
     }
 
     /// Reads a single parameter from the sensor.
-    /// `parser` must be created with `crate::parameter::ReadParam::new_parser()`.
+    ///
+    /// `parser` must be created with
+    /// [`ReadParam::new_parser()`](crate::parameter::ReadParam), which requires
+    /// [`InitParser`](crate::parse_result::InitParser) to be in scope, and the
+    /// sensor must already be in configuration mode
+    /// ([`begin_config`](Self::begin_config)).
+    ///
+    /// The value is returned raw; threshold parameters must be converted with
+    /// [`decode_threschold_value`](crate::decode_threschold_value) to obtain dB.
+    /// `Ok(None)` means the parser carried no decoder.
     pub async fn get_param_value<
     'a,
     DECODER,
@@ -156,14 +184,18 @@ DELAY: DelayNs,
     }
 
 
-    /// Configures the maximum range and delay, loads the default trigger/hold
-    /// thresholds for all 16 distance gates, saves the configuration, and puts
-    /// the sensor into report mode.
+    /// Configures the maximum range and the absence delay, then writes the
+    /// factory-default trigger/hold thresholds for all 16 distance gates and
+    /// saves the configuration.
     ///
-    /// All parameters are written between `begin_config` and `end_save_config`.
+    /// `max_range` is a gate index (`0..=15`, 70 cm per gate); `delay_sec` is
+    /// the absence report delay in seconds. Every parameter must be written
+    /// before `end_save_config()` for the configuration to reach flash.
     ///
-    /// **Note:** every parameter must be written before `end_save_config()` for
-    /// the configuration to be persisted to the sensor's flash.
+    /// This does **not** switch the sensor into a report mode — call
+    /// [`set_report_mode_35byte_payload`](Self::set_report_mode_35byte_payload)
+    /// afterwards. Unlike the rest of this type, the method returns `()` and
+    /// discards every error.
     #[allow(dead_code)]
     pub async fn set_range_delay_with_default_thresholds(&mut self, max_range: u8, delay_sec: u32){
 
@@ -213,6 +245,12 @@ DELAY: DelayNs,
 
     }
 
+    /// Writes a batch of parameters, keeping every parameter not listed in
+    /// `params_values` at the value currently stored in the sensor (falling
+    /// back to [`ParameterID::default_value`] if it cannot be read).
+    ///
+    /// Thresholds are given in dB; `RangeGate` and `AbsenseReportDelay` as raw
+    /// values. Returns `()` and discards every error.
     #[allow(dead_code)]
     pub async fn set_params_value(&mut self, params_values: &[(ParameterID, f32)]){
 
diff --git a/src/report_debug_mode.rs b/src/report_debug_mode.rs
index 745413c..7eb978f 100644
--- a/src/report_debug_mode.rs
+++ b/src/report_debug_mode.rs
@@ -1,4 +1,17 @@
 //! Debug reporting mode: the range-Doppler map (RDMAP) frame and its command.
+//!
+//! Command value `00 00 00 00`. Frames are 1288 bytes: header `AA BF 10 14`
+//! (`10` = 16 range gates, `14` = 20 Doppler bins), 1280 bytes of payload, and
+//! the tail `FD FC FB FA` — which happens to be the same byte sequence as
+//! [`SEND_HEADER`](crate::SEND_HEADER).
+//!
+//! # Known issue
+//!
+//! `EXPECTED_CMD_ID` is set to [`CommandID::ReportMode`], but a debug frame
+//! carries **no command-id field**. The parser therefore consumes the first two
+//! payload bytes as a command id, compares them against `0x1200` and resets.
+//! Debug frames will not parse until this constant is changed to
+//! [`CommandID::None`].
 use super::{Parser, PayloadDecoder, CommandID};
 
 const CMD_HEADER: [u8; 4] = [0xAA, 0xBF, 0x10, 0x14];
@@ -12,6 +25,9 @@ const EXPECTED_CMD_ID: u16 = COMMAND_ID.as_u16();
 const RESERVED_LEN: usize = 0;
 const HAS_DATA_LENGHT_BYTES: bool = false;
 
+/// Decodes a 1280-byte RDMAP payload into an [`HmmdRdmapFrame`].
+///
+/// Returns a zero-filled frame if the payload length is unexpected.
 pub struct Decoder;
 
 type DecoderType = Decoder;
@@ -53,9 +69,13 @@ impl PayloadDecoder for DecoderType {
     }
 }
 
-/// A range-Doppler map frame (debug mode): a 20 (Doppler) × 16 (range gate)
-/// matrix where each cell is the squared amplitude as a `u32`.
+/// A range-Doppler map frame (debug mode).
+///
+/// Build a parser for it with `HmmdRdmapFrame::new_parser()`
+/// ([`InitParser`](crate::parse_result::InitParser) must be in scope).
 pub struct HmmdRdmapFrame {
+    /// 20 Doppler bins × 16 range gates; each cell is the squared amplitude,
+    /// little-endian `u32` on the wire. Indexed `rdmap[doppler][gate]`.
     pub rdmap: [[u32; 16]; 20],
 }
 
@@ -74,56 +94,13 @@ impl<'a> super::parse_result::InitParser<'a, DecoderType, PAYLOAD_LEN, RESERVED_
 
 
 
-/*
-impl <'a>PayloadDecoder<'a, PAYLOAD_LEN,  RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES, HmmdRdmapFrame> for HmmdRdmapFrame {
-
-    /// Builds a parser configured for the 1280-byte RDMAP frame.
-    fn new_parser(&self) -> ParserType<'a> {
-        ParserType::new(&CMD_HEADER, &CMD_TAIL)
-    }
-
-    /// Decodes the 1280-byte payload (320 little-endian `u32` values) into the
-    /// 20×16 range-Doppler matrix. Returns a zero-filled frame if the payload
-    /// length is unexpected.
-    fn decode(&self, payload:&[u8]) -> Self{
-
-        if payload.len() != PAYLOAD_LEN {
-            return  Self {
-                rdmap:[[0u32; 16]; 20]
-            };
-        }
-
-        let mut rdmap = [[0u32; 16]; 20];
-
-        let mut index = 0;
-
-        for doppler in &mut rdmap {
-
-            for gate in doppler.iter_mut().take(16) {
-
-                *gate = u32::from_le_bytes([
-                        payload[index],
-                        payload[index + 1],
-                        payload[index + 2],
-                        payload[index + 3],
-                ]);
-
-                index += 4;
-            }
-        }
-
-        Self {
-            rdmap
-        }
-
-    }
-
-}
-*/
 
 use super::{SerialCmd, SEND_HEADER, SEND_TAIL};
 
 /// Frame builder for switching the sensor into debug mode.
+///
+/// See the module documentation for the frame layout and the known parser
+/// issue. The command defines **no ACK**, so nothing is verified after sending.
 //send FD FC FB FA 08 00 12 00 00 00 00 00 00 00 04 03 02 01
 //result: 1280
 /*
diff --git a/src/report_normal_mode.rs b/src/report_normal_mode.rs
index 601618b..2867c39 100644
--- a/src/report_normal_mode.rs
+++ b/src/report_normal_mode.rs
@@ -1,4 +1,9 @@
-//! Normal reporting mode: the per-frame detection result and its command.
+//! Binary reporting mode: the per-frame detection result and its command.
+//!
+//! This module covers what the Waveshare wiki calls **Report Mode** (command
+//! value `04 00 00 00`): a 45-byte frame carrying presence, target distance and
+//! 16 per-gate energies. The wiki's own "Normal Mode" (`64 00 00 00`) is the
+//! ASCII `ON <gate>` / `OFF` output and is *not* implemented here.
 
 use super::{Parser, PayloadDecoder};
 
@@ -13,6 +18,7 @@ const RESERVED_LEN: usize = 0;
 const HAS_DATA_LENGHT_BYTES: bool = true;
 
 
+/// Decodes a 35-byte report payload into an [`HmmdFrame`].
 pub struct Decoder;
 
 type DecoderType = Decoder;
@@ -42,11 +48,19 @@ impl PayloadDecoder for Decoder {
 }
 
 
-/// A normal-mode report frame: target presence, distance, and per-gate energies.
+/// A decoded report frame: target presence, distance, and per-gate energies.
+///
+/// Build a parser for it with `HmmdFrame::new_parser()`
+/// ([`InitParser`](crate::parse_result::InitParser) must be in scope).
 pub struct HmmdFrame{
 
+    /// `true` when a target is present (status byte `01`).
     pub present: bool,
+    /// Straight-line distance from the radar to the target, in centimetres.
     pub distance_cm: u16,
+    /// Raw energy of each of the 16 distance gates, gate `0` first.
+    /// Convert to dB with
+    /// [`decode_threschold_value`](crate::decode_threschold_value).
     pub energy: [u16; 16],
 }
 
@@ -68,11 +82,17 @@ impl<'a> super::parse_result::InitParser<'a, DecoderType, PAYLOAD_LEN, RESERVED_
 
 use super::{SerialCmd, CommandID, SEND_HEADER, SEND_TAIL};
 
-/// Frame builder for switching the sensor into normal report mode.
-///
-/// After this command the sensor streams 45-byte frames (35-byte payload):
-/// 1 status byte (`00` absent, `01` present), 2 bytes target distance, and
-/// 16 × 2 bytes of per-gate energy.
+/// Frame builder for switching the sensor into binary report mode.
+impl SerialCmd<18, 0>{
+    /// Builds the command that puts the sensor into Waveshare's *Report Mode*
+    /// (`04 00 00 00`).
+    ///
+    /// After this command the sensor streams 45-byte frames every 100 ms:
+    /// header `F4 F3 F2 F1`, a 2-byte length (`23 00` = 35), a 35-byte payload
+    /// — 1 status byte (`00` absent, `01` present), 2 bytes target distance in
+    /// cm, and 16 × 2 bytes of per-gate energy — then tail `F8 F7 F6 F5`.
+    ///
+    /// The command defines **no ACK**, so nothing is verified after sending.
 //send FD FC FB FA 08 00 12 00 00 00 04 00 00 00 04 03 02 01
 //result: 45 byte
 //F4 F3 F2 F1
@@ -82,7 +102,6 @@ use super::{SerialCmd, CommandID, SEND_HEADER, SEND_TAIL};
 //DA A3 C9 D8 39 08 12 00 28 00 94 00 44 00 91 00 31 00 7A 00 6D 00 52 00 6D 00 35 00 65 00 41 00
 //16 (total number of distance gates) * 2 bytes, size of energy value for each distance gate from 0 to 15
 //F8 F7 F6 F5
-impl SerialCmd<18, 0>{
     pub fn set_report_mode() -> Self{
 
         let cmd_id_2b = CommandID::ReportMode.get_bytes();
diff --git a/src/send_models.rs b/src/send_models.rs
index a8784d7..c534416 100644
--- a/src/send_models.rs
+++ b/src/send_models.rs
@@ -11,8 +11,23 @@ use super::{CommandID, SEND_HEADER, SEND_TAIL};
 /// sensor's response (`0` when the command returns no ACK payload).
 pub struct SerialCmd<const S: usize, const R: usize>{
 
+    /// The complete frame to write to the UART, header and tail included.
     pub send: [u8; S],
+    /// The ACK payload to compare the sensor's reply against — command id,
+    /// status word and any returned data, without header, length or tail.
+    /// Empty when the command defines no ACK.
     pub result_payload_ack: [u8; R],
+    /// How long to wait after writing the frame before reading the reply.
+    ///
+    /// **The unit is not nanoseconds despite the name.** The values used by
+    /// the builders are `2000` for configuration commands and `100` for
+    /// report-mode commands, which only make sense as microseconds or larger —
+    /// at 115200 baud one byte alone takes ~87 µs. [`MicrowaveRadar`](crate::MicrowaveRadar)
+    /// passes it to a `delay_us`-named method, while
+    /// [`MicrowaveRadarIO`](crate::MicrowaveRadarIO) and
+    /// [`MicrowaveRadarIOAsync`](crate::MicrowaveRadarIOAsync) pass it to
+    /// `DelayNs::delay_ns`. Treat the field as microseconds and scale it in
+    /// your delay implementation accordingly.
     pub delay_ns: u32,
 
 }
@@ -90,7 +105,9 @@ impl SerialCmd<12, 4>{
 
 /// Encodes a threshold value (in dB) into 4 little-endian bytes.
 ///
-/// Applies `10^(value / 10)` and truncates to `u32`.
+/// Applies `10^(value / 10)` and truncates to `u32`. Returns four zero bytes
+/// for an input of `0.0`. Inverse of
+/// [`decode_threschold_value`](crate::decode_threschold_value).
 pub fn encode_threshold_value_to_le_bytes(value: f32) -> [u8; 4] {
     if value == 0.0 {
         return [0x00, 0x00, 0x00, 0x00];