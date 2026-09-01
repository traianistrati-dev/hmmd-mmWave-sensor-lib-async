//! MicrowaveRadarAsync built on the standard `embedded-io-async` /
//! `embedded-hal-async` traits — no closures, and no custom
//! `UsartTx` / `UsartRx` / `Delay` traits.
//!
//! Cargo.toml:
//!   embedded-io-async  = "0.6"
//!   embedded-hal-async = "1.0"

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, ReadExactError, Write};

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

/// Driver generic over any TX/RX pair implementing the standard async I/O
/// traits. On esp-hal: `let (rx, tx) = uart.split();` — both halves implement
/// `Read`/`Write` when the driver is in `Async` mode.
pub struct MicrowaveRadarAsync<TX, RX, DELAY> {
    tx: TX,
    rx: RX,
    delay: DELAY,
}

impl<TX, RX, DELAY> MicrowaveRadarAsync<TX, RX, DELAY>
where
TX: Write,
RX: Read,
DELAY: DelayNs,
{
    ///delay suported as parameter:
    /// embassy_time::Delay
    /// embedded_hal_async::delay::DelayNs
    pub fn new(tx: TX, rx: RX, delay: DELAY) -> Self {
        Self { tx, rx, delay }
    }

    /// Sends one complete frame and waits until the FIFO has drained.
    /// `write_all` loops over partial writes for us — no manual retry needed.
    async fn write_cmd(&mut self, data: &[u8]) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.tx.write_all(data).await.map_err(RadarError::Tx)?;
        self.tx.flush().await.map_err(RadarError::Tx)
    }

    /// One byte of the reply. `read_exact` waits until a byte actually arrives.
    pub async fn next_byte(&mut self) -> Result<u8, RadarError<TX::Error, RX::Error>> {
        let mut b = [0u8; 1];
        self.rx.read_exact(&mut b).await?;
        Ok(b[0])
    }

    // pub async fn next_bytes(&mut self, buff: &mut [u8]) -> Result<(), RadarError<TX::Error, RX::Error>> {
        // self.rx.read_exact(buff).await?;
        // Ok(())
    // }

    /// Sends a command, then feeds the parser until a valid frame is decoded.
    pub async fn send_cmd_and_get_result<
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
        self.write_cmd(&data.send).await?;
        self.delay.delay_us(data.delay_us).await;

        loop {
            let b = self.next_byte().await?;
            if parser.feed(b) {
                return Ok(parser.decode_payload());
            }
        }
    }

    /// Sends a command and verifies the ACK payload.
    pub async fn send_cmd_and_check_ack_result<const S: usize, const R: usize>(
        &mut self,
        data: SerialCmd<S, R>,
    ) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.write_cmd(&data.send).await?;
        self.delay.delay_us(data.delay_us).await;

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
            let b = self.next_byte().await?;
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
    pub async fn begin_config(&mut self) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.send_cmd_and_check_ack_result(SerialCmd::begin_config()).await
    }

    /// Persists the configuration to the sensor's flash ("end/save config").
    pub async fn end_save_config(&mut self) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.send_cmd_and_check_ack_result(SerialCmd::end_save_config()).await
    }

    /// High-level helpers are async too, and propagate the error with `?`
    /// instead of swallowing it into a `bool`.
    pub async fn set_report_mode_35byte_payload(&mut self) -> Result<(), RadarError<TX::Error, RX::Error>> {
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode()).await
    }

    /// Reads a single parameter from the sensor.
    /// `parser` must be created with `crate::parameter::ReadParam::new_parser()`.
    pub async fn get_param_value<
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
        self.send_cmd_and_get_result(SerialCmd::read_param_value(param_id), parser).await
    }
}