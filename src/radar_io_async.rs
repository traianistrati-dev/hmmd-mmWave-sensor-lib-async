//! MicrowaveRadarAsync built on the standard `embedded-io-async` /
//! `embedded-hal-async` traits — no closures, and no custom
//! `UsartTx` / `UsartRx` / `Delay` traits.
//!
//! Cargo.toml:
//!   embedded-io-async  = "0.6"
//!   embedded-hal-async = "1.0"

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, Write};

use super::{ParameterID, PayloadDecoder, SerialCmd, RadarError, RadarIoResult};


/// Driver generic over any TX/RX pair implementing the standard async I/O
/// traits. On esp-hal: `let (rx, tx) = uart.split();` — both halves implement
/// `Read`/`Write` when the driver is in `Async` mode.
pub struct MicrowaveRadarIOAsync<TX, RX, DELAY> {
    tx: TX,
    rx: RX,
    delay: DELAY,
}

impl<TX, RX, DELAY> MicrowaveRadarIOAsync<TX, RX, DELAY>
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
    async fn write_cmd(&mut self, data: &[u8]) -> RadarIoResult<(), TX, RX>{
        self.tx.write_all(data).await.map_err(RadarError::Tx)?;
        self.tx.flush().await.map_err(RadarError::Tx)
    }

    /// One byte of the reply. `read_exact` waits until a byte actually arrives.
    pub async fn next_byte(&mut self) -> RadarIoResult<u8, TX, RX> {
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
    ) -> RadarIoResult<Option<DECODER::Output>, TX, RX>
    where DECODER: PayloadDecoder,
    {
        self.write_cmd(&data.send).await?;
        self.delay.delay_ns(data.delay_ns).await;

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
    ) -> RadarIoResult<(), TX, RX>
    {
        self.write_cmd(&data.send).await?;
        self.delay.delay_ns(data.delay_ns).await;

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
    pub async fn begin_config(&mut self) -> RadarIoResult<(), TX, RX> {
        self.send_cmd_and_check_ack_result(SerialCmd::begin_config()).await
    }

    /// Persists the configuration to the sensor's flash ("end/save config").
    pub async fn end_save_config(&mut self) -> RadarIoResult<(), TX, RX>  {
        self.send_cmd_and_check_ack_result(SerialCmd::end_save_config()).await
    }

    /// High-level helpers are async too, and propagate the error with `?`
    /// instead of swallowing it into a `bool`.
    #[allow(dead_code)]
    pub async fn set_report_mode_35byte_payload(&mut self) -> RadarIoResult<(), TX, RX>{
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode()).await
    }

    #[allow(dead_code)]
    pub async fn set_report_debug_mode_1280byte_payload(&mut self) -> RadarIoResult<(), TX, RX>{
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode()).await
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
    ) -> RadarIoResult<Option<DECODER::Output>, TX, RX>
    where
    DECODER: PayloadDecoder,
    {
        self.send_cmd_and_get_result(SerialCmd::read_param_value(param_id), parser).await
    }


    /// Configures the maximum range and delay, loads the default trigger/hold
    /// thresholds for all 16 distance gates, saves the configuration, and puts
    /// the sensor into report mode.
    ///
    /// All parameters are written between `begin_config` and `end_save_config`.
    ///
    /// **Note:** every parameter must be written before `end_save_config()` for
    /// the configuration to be persisted to the sensor's flash.
    #[allow(dead_code)]
    pub async fn set_range_delay_with_default_thresholds(&mut self, max_range: u8, delay_sec: u32){

        if self.begin_config().await.is_ok() || self.begin_config().await.is_ok() {

            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, max_range as f32)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, delay_sec as f32)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, ParameterID::TriggerThreshold00.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ParameterID::HoldThreshold00.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, ParameterID::TriggerThreshold01.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ParameterID::HoldThreshold01.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, ParameterID::TriggerThreshold02.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ParameterID::HoldThreshold02.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, ParameterID::TriggerThreshold03.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ParameterID::HoldThreshold03.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, ParameterID::TriggerThreshold04.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ParameterID::HoldThreshold04.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, ParameterID::TriggerThreshold05.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ParameterID::HoldThreshold05.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, ParameterID::TriggerThreshold06.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ParameterID::HoldThreshold06.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, ParameterID::TriggerThreshold07.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ParameterID::HoldThreshold07.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, ParameterID::TriggerThreshold08.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ParameterID::HoldThreshold08.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, ParameterID::TriggerThreshold09.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ParameterID::HoldThreshold09.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, ParameterID::TriggerThreshold10.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ParameterID::HoldThreshold10.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, ParameterID::TriggerThreshold11.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ParameterID::HoldThreshold11.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, ParameterID::TriggerThreshold12.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ParameterID::HoldThreshold12.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, ParameterID::TriggerThreshold13.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ParameterID::HoldThreshold13.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, ParameterID::TriggerThreshold14.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ParameterID::HoldThreshold14.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, ParameterID::TriggerThreshold15.default_value())).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ParameterID::HoldThreshold15.default_value())).await;


            // Do something after saving the configuration
            _ = self.end_save_config().await;
        }

        // self.set_report_mode_35byte_payload();

    }

    #[allow(dead_code)]
    pub async fn set_params_value(&mut self, params_values: &[(ParameterID, f32)]){

        use super::parse_result::InitParser;
        let mut parser_params = super::parameter::ReadParam::new_parser();

        let mut value_for_param = async|target: ParameterID| -> f32 {


            for (param_id, value) in params_values{

                if param_id == &target {
                    return *value;
                }
            }


            match self.get_param_value(target, &mut parser_params).await {
                Ok(Some(v)) => v as f32,
                _ => target.default_value(),
            }
        };

        let range_gate = value_for_param(ParameterID::RangeGate).await;
        let absense_delay = value_for_param(ParameterID::AbsenseReportDelay).await;

        let tt00 = value_for_param(ParameterID::TriggerThreshold00).await;
        let ht00 = value_for_param(ParameterID::HoldThreshold00).await;

        let tt01 = value_for_param(ParameterID::TriggerThreshold01).await;
        let ht01 = value_for_param(ParameterID::HoldThreshold01).await;

        let tt02 = value_for_param(ParameterID::TriggerThreshold02).await;
        let ht02 = value_for_param(ParameterID::HoldThreshold02).await;

        let tt03 = value_for_param(ParameterID::TriggerThreshold03).await;
        let ht03 = value_for_param(ParameterID::HoldThreshold03).await;

        let tt04 = value_for_param(ParameterID::TriggerThreshold04).await;
        let ht04 = value_for_param(ParameterID::HoldThreshold04).await;

        let ht05 = value_for_param(ParameterID::HoldThreshold06).await;
        let tt05 = value_for_param(ParameterID::TriggerThreshold05).await;

        let tt06 = value_for_param(ParameterID::TriggerThreshold06).await;
        let ht06 = value_for_param(ParameterID::HoldThreshold06).await;

        let tt07 = value_for_param(ParameterID::TriggerThreshold07).await;
        let ht07 = value_for_param(ParameterID::HoldThreshold07).await;

        let tt08 = value_for_param(ParameterID::TriggerThreshold08).await;
        let ht08 = value_for_param(ParameterID::HoldThreshold08).await;

        let tt09 = value_for_param(ParameterID::TriggerThreshold09).await;
        let ht09 = value_for_param(ParameterID::HoldThreshold09).await;

        let tt10 = value_for_param(ParameterID::TriggerThreshold10).await;
        let ht10 = value_for_param(ParameterID::HoldThreshold10).await;

        let tt11 = value_for_param(ParameterID::TriggerThreshold11).await;
        let ht11 = value_for_param(ParameterID::HoldThreshold11).await;

        let tt12 = value_for_param(ParameterID::TriggerThreshold12).await;
        let ht12 = value_for_param(ParameterID::HoldThreshold12).await;

        let tt13 = value_for_param(ParameterID::TriggerThreshold13).await;
        let ht13 = value_for_param(ParameterID::HoldThreshold13).await;

        let tt14 = value_for_param(ParameterID::TriggerThreshold14).await;
        let ht14 = value_for_param(ParameterID::HoldThreshold14).await;

        let tt15 = value_for_param(ParameterID::TriggerThreshold15).await;
        let ht15 = value_for_param(ParameterID::HoldThreshold15).await;



        if self.begin_config().await.is_ok() || self.begin_config().await.is_ok(){

            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, range_gate)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, absense_delay)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, tt00)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ht00)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, tt01)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ht01)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, tt02)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ht02)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, tt03)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ht03)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, tt04)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ht04)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, tt05)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ht05)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, tt06)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ht06)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, tt07)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ht07)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, tt08)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ht08)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, tt09)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ht09)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, tt10)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ht10)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, tt11)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ht11)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, tt12)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ht12)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, tt13)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ht13)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, tt14)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ht14)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, tt15)).await;
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ht15)).await;

            _ = self.end_save_config().await;
        }


    }



}