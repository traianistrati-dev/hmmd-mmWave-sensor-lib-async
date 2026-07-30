
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
//!   [`ReadParam`](parameter::ReadParam)) implement [`ParserResult`] to
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

pub mod data;
pub mod send_models;
pub mod parse_result;
pub mod parameter;
pub mod radar;
pub mod report_debug_mode;
pub mod report_normal_mode;


pub use radar::*;
pub use data::*;
pub use send_models::*;
pub use parse_result::*;
pub use report_debug_mode::*;
