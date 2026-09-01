
//! `no_std` driver for the HMMD mmWave presence sensor over its USART protocol.
//!
//! This crate builds and parses the HMMD serial protocol frames: configuration
//! commands, parameter read/write, and the two reporting modes (normal and
//! debug/RDMAP). It is HAL-agnostic — you talk to the sensor through three
//! small traits ([`UsartTx`], [`UsartRx`], [`Delay`]), which are implemented
//! automatically for closures, so it works on any chip.
//!
//! # Overview
//!
//! - [`MicrowaveRadar`] is the high-level entry point: it sends commands, waits,
//!   and validates ACKs or decodes responses.
//! - [`SerialCmd`] represents a ready-to-send command frame together with its
//!   expected ACK payload.
//! - [`Parser`] is an incremental, byte-by-byte frame parser; concrete frame
//!   types ([`HmmdFrame`](report_normal_mode::HmmdFrame), [`HmmdRdmapFrame`],
//!   [`ReadParam`](parameter::ReadParam)) implement [`PayloadDecoder`] to
//!   configure and decode it.
//!
//! # Example
//!
//! ```ignore
//! let delay_us = |us: u32| { /* busy-wait `us` microseconds */ };
//! let tx = |bytes: &[u8]| { /* write bytes to the UART */ };
//! let rx = || -> Option<u8> { /* read one byte if available */ None };
//!
//! let mut radar = hmmd_mmwave_sensor::MicrowaveRadar::new(delay_us, tx, rx);
//!
//! let mut parser = hmmd_mmwave_sensor::parameter::ReadParam::new_parser();
//! let range = radar.get_param_value(
//!     hmmd_mmwave_sensor::data::ParameterID::RangeGate,
//!     &mut parser,
//! );
//! ```

#![no_std]

pub mod send_models;
pub mod parse_result;
pub mod parameter;
pub mod radar;
pub mod radar_io;
pub mod radar_io_async;
pub mod report_debug_mode;
pub mod report_normal_mode;


pub use radar::*;
pub use radar_io::*;
pub use radar_io_async::*;
pub use send_models::*;
pub use parse_result::*;
pub use report_debug_mode::*;

// pub mod examples;



/// 4-byte header prefixing every command frame sent to the sensor.
pub const SEND_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
/// 4-byte tail terminating every command frame sent to the sensor.
pub const SEND_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];

pub const BAUD_RATE: u32 = 115200;


/// Protocol constants: parameter/command identifiers and frame delimiters.
/// Identifiers of the sensor's configurable parameters.
///
/// Each variant's value is the 16-bit word placed on the wire (big-endian via
/// [`get_bytes`](ParameterID::get_bytes)).
///
/// Range is expressed in *gates*: each gate spans ~70 cm, so `RangeGate = N`
/// configures a maximum detection distance of roughly `N * 70 cm`
/// (e.g. `RangeGate = 0` ≈ 0–70 cm, `RangeGate = 1` ≈ 0–140 cm). For a given
/// `RangeGate = N`, the trigger/hold thresholds of gates `00..=N` set the
/// per-gate detection sensitivity.
#[repr(u16)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ParameterID{
    /// Maximum detection range, in gates (~70 cm per gate).
    RangeGate = 0x0100,
    /// Absence report delay, in seconds, before a target is reported as gone.
    AbsenseReportDelay = 0x0400,

    TriggerThreshold00 = 0x1000,
    TriggerThreshold01 = 0x1100,
    TriggerThreshold02 = 0x1200,
    TriggerThreshold03 = 0x1300,
    TriggerThreshold04 = 0x1400,
    TriggerThreshold05 = 0x1500,
    TriggerThreshold06 = 0x1600,

    TriggerThreshold07 = 0x1700,
    TriggerThreshold08 = 0x1800,
    TriggerThreshold09 = 0x1900,

    TriggerThreshold10 = 0x1A00,
    TriggerThreshold11 = 0x1B00,
    TriggerThreshold12 = 0x1C00,

    TriggerThreshold13 = 0x1D00,
    TriggerThreshold14 = 0x1E00,
    TriggerThreshold15 = 0x1F00,


    HoldThreshold00 = 0x2000,
    HoldThreshold01 = 0x2100,
    HoldThreshold02 = 0x2200,

    HoldThreshold03 = 0x2300,
    HoldThreshold04 = 0x2400,
    HoldThreshold05 = 0x2500,

    HoldThreshold06 = 0x2600,
    HoldThreshold07 = 0x2700,
    HoldThreshold08 = 0x2800,

    HoldThreshold09 = 0x2900,
    HoldThreshold10 = 0x2A00,
    HoldThreshold11 = 0x2B00,

    HoldThreshold12 = 0x2C00,
    HoldThreshold13 = 0x2D00,
    HoldThreshold14 = 0x2E00,
    HoldThreshold15 = 0x2F00,

}

impl ParameterID{
    /// Serializes the identifier into its 2 big-endian bytes as sent in a command frame.
    pub fn get_bytes(self) -> [u8; 2]{
        (self as u16).to_be_bytes()
    }
    /// Returns the manufacturer default value for this parameter.
    ///
    /// `RangeGate` and `AbsenseReportDelay` are returned as raw values; the
    /// trigger/hold thresholds are returned in dB.
    pub fn default_value(&self) -> f32{
        match self {
            ParameterID::RangeGate => 15.0,
            ParameterID::AbsenseReportDelay => 10.0,
            ParameterID::TriggerThreshold00 => 48.93,
            ParameterID::TriggerThreshold01 => 45.57,
            ParameterID::TriggerThreshold02 => 43.20,
            ParameterID::TriggerThreshold03 => 36.18,
            ParameterID::TriggerThreshold04 => 34.45,
            ParameterID::TriggerThreshold05 => 32.04,
            ParameterID::TriggerThreshold06 => 30.22,
            ParameterID::TriggerThreshold07 => 27.90,
            ParameterID::TriggerThreshold08 => 25.86,
            ParameterID::TriggerThreshold09 => 23.45,
            ParameterID::TriggerThreshold10 => 21.90,
            ParameterID::TriggerThreshold11 => 21.37,
            ParameterID::TriggerThreshold12 => 19.98,
            ParameterID::TriggerThreshold13 => 20.05,
            ParameterID::TriggerThreshold14 => 18.98,
            ParameterID::TriggerThreshold15 => 18.75,
            ParameterID::HoldThreshold00 => 47.38,
            ParameterID::HoldThreshold01 => 44.03,
            ParameterID::HoldThreshold02 => 41.66,
            ParameterID::HoldThreshold03 => 34.63,
            ParameterID::HoldThreshold04 => 32.90,
            ParameterID::HoldThreshold05 => 30.49,
            ParameterID::HoldThreshold06 => 28.67,
            ParameterID::HoldThreshold07 => 26.35,
            ParameterID::HoldThreshold08 => 24.31,
            ParameterID::HoldThreshold09 => 21.90,
            ParameterID::HoldThreshold10 => 20.35,
            ParameterID::HoldThreshold11 => 19.82,
            ParameterID::HoldThreshold12 => 18.44,
            ParameterID::HoldThreshold13 => 18.50,
            ParameterID::HoldThreshold14 => 17.43,
            ParameterID::HoldThreshold15 => 17.20,
        }
    }

    /// Returns the command code as a `u16`
    pub const fn as_u16(self) -> u16 {
        self as u16
    }

}

/// Command codes of the HMMD protocol, including the matching ACK codes.
///
/// [`None`](CommandID::None) is a sentinel used by [`Parser`](crate::Parser) to
/// mean "this frame type carries no command-id field".
#[repr(u16)]
pub enum CommandID{
    EnableConfig = 0xFF00,
    EnableConfigAck = 0xFF01,
    EndSaveConfig = 0xFE00,
    EndSaveConfigAck = 0xFE01,
    WriteParam = 0x0700,
    WriteParamAck = 0x0701,
    ReadParam = 0x0800,
    ReadParamAck = 0x0801,
    ReadFirmwareVersion = 0x0000,
    ReadSerialNumber = 0x1100,
    ReportMode = 0x1200, //64 00 00 00 = Basic (ASCII ON RangeGate 1234 or OFF); 04 00 00 00 = RangeGate with energy; 00 00 00 00 = 20Dopple * 16EnergyGates
    None = 0xFFFF,
}

impl CommandID{
    /// Serializes the command code into its 2 big-endian bytes.f
    pub fn get_bytes(self) -> [u8; 2]{
        (self as u16).to_be_bytes()
    }
    /// Returns the command code as a `u16`, usable in `const` context
    /// (e.g. as a const generic argument of [`Parser`](crate::Parser)).
    pub const fn as_u16(self) -> u16 {
        self as u16
    }

}













