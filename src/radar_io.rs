//! `MicrowaveRadar` built on the standard blocking `embedded-io` /
//! `embedded-hal` traits — no closures, and no custom
//! `UsartTx` / `UsartRx` / `Delay` traits.
//!
//! Mirror of `radar_async.rs`: same API, same error type, no `async`/`.await`.
//!
//! Cargo.toml:
//!   embedded-io  = "0.6"
//!   embedded-hal = "1.0"

use embedded_hal::delay::DelayNs;
use embedded_io::{Read, ReadExactError, Write};

use super::{ParameterID, PayloadDecoder, SerialCmd};

/// Everything that can go wrong while talking to the sensor.
#[derive(Debug)]
pub enum RadarError<TxErr, RxErr> {
    /// Writing the command frame to the UART failed.
    Tx(TxErr),
    /// Reading the reply failed.
    Rx(RxErr),
    /// The link closed before a byte arrived (EOF).
    Eof,
    /// A reply arrived, but it does not match the expected ACK payload.
    UnexpectedAck,
}

impl<TxErr, RxErr> From<ReadExactError<RxErr>> for RadarError<TxErr, RxErr> {
    fn from(e: ReadExactError<RxErr>) -> Self {
        match e {
            ReadExactError::UnexpectedEof => RadarError::Eof,
            ReadExactError::Other(e) => RadarError::Rx(e),
        }
    }
}

/// Driver generic over any TX/RX pair implementing the standard blocking I/O
/// traits. On esp-hal: `let (rx, tx) = uart.split();` — both halves implement
/// `Read`/`Write` when the driver is in `Blocking` mode.
///
/// Note: every call here busy-waits until the exchange completes. Under an
/// async runtime this stalls the whole executor — use `MicrowaveRadarAsync`
/// there instead.
pub struct MicrowaveRadar<TX, RX, DELAY> {
    tx: TX,
    rx: RX,
    delay: DELAY,
}

impl<TX, RX, DELAY> MicrowaveRadar<TX, RX, DELAY>
where
    TX: Write,
    RX: Read,
    DELAY: DelayNs,
{
    /// Delays supported as a parameter:
    /// `esp_hal::delay::Delay`
    /// any `embedded_hal::delay::DelayNs`
    pub fn new(tx: TX, rx: RX, delay: DELAY) -> Self {
        Self { tx, rx, delay }
    }

    /// Sends one complete frame and waits until the FIFO has drained.
    /// `write_all` loops over partial writes for us — no manual retry needed.
    fn write_cmd(&mut self, data: &[u8]) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.tx.write_all(data).map_err(RadarError::Tx)?;
        self.tx.flush().map_err(RadarError::Tx)
    }

    /// One byte of the reply. `read_exact` blocks until a byte actually arrives.
    fn next_byte(&mut self) -> Result<u8, RadarError<TX::Error, RX::Error>> {
        let mut b = [0u8; 1];
        self.rx.read_exact(&mut b)?;
        Ok(b[0])
    }

    /// Sends a command, then feeds the parser until a valid frame is decoded.
    pub fn send_cmd_and_get_result<
        'a,
        DECODER,
        const S: usize,
        const PAYLOAD_LEN: usize,
        const RESERVED_LEN: usize,
        const EXPECTED_CMD_ID: u16,
        const HAS_DATA_LENGHT_BYTES: bool,
    >(
        &mut self,
        data: SerialCmd<S, 0>,
        parser: &mut super::Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>,
    ) -> Result<Option<DECODER::Output>, RadarError<TX::Error, RX::Error>>
    where
        DECODER: PayloadDecoder,
    {
        self.write_cmd(&data.send)?;
        self.delay.delay_us(data.delay_us);

        loop {
            let b = self.next_byte()?;
            if parser.feed(b) {
                return Ok(parser.decode_payload());
            }
        }
    }

    /// Sends a command and verifies the ACK payload.
    pub fn send_cmd_and_check_ack_result<const S: usize, const R: usize>(
        &mut self,
        data: SerialCmd<S, R>,
    ) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.write_cmd(&data.send)?;
        self.delay.delay_us(data.delay_us);

        // Commands with no ACK payload are considered done once sent.
        if data.result_payload_ack.is_empty() {
            return Ok(());
        }

        let mut parser = super::Parser::<(), R, 0, { super::CommandID::None.as_u16() }, true>::new(
            &super::SEND_HEADER,
            &super::SEND_TAIL,
            None,
        );

        loop {
            let b = self.next_byte()?;
            if parser.feed(b) {
                for i in 0..R {
                    if data.result_payload_ack[i] != parser.payload[i] {
                        return Err(RadarError::UnexpectedAck);
                    }
                }
                return Ok(());
            }
        }
    }

    /// Enters configuration mode ("enable config" command).
    pub fn begin_config(&mut self) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.send_cmd_and_check_ack_result(SerialCmd::begin_config())
    }

    /// Persists the configuration to the sensor's flash ("end/save config").
    pub fn end_save_config(&mut self) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.send_cmd_and_check_ack_result(SerialCmd::end_save_config())
    }

    /// High-level helpers propagate the error with `?` instead of swallowing
    /// it into a `bool`.
    pub fn set_report_mode_35byte_payload(&mut self) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode())
    }

    /// Reads a single parameter from the sensor.
    /// `parser` must be created with `crate::parameter::ReadParam::new_parser()`.
    pub fn get_param_value<
        'a,
        DECODER,
        const PAYLOAD_LEN: usize,
        const RESERVED_LEN: usize,
        const EXPECTED_CMD_ID: u16,
        const HAS_DATA_LENGHT_BYTES: bool,
    >(
        &mut self,
        param_id: ParameterID,
        parser: &mut super::Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>,
    ) -> Result<Option<DECODER::Output>, RadarError<TX::Error, RX::Error>>
    where
        DECODER: PayloadDecoder,
    {
        self.send_cmd_and_get_result(SerialCmd::read_param_value(param_id), parser)
    }
}