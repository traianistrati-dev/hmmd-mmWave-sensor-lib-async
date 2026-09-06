//! `MicrowaveRadar` built on the standard blocking `embedded-io` /
//! `embedded-hal` traits — no closures, and no custom
//! `UsartTx` / `UsartRx` / `Delay` traits.
//!
//! Mirror of `radar_.rs`: same API, same error type, no ``/``.
//!
//! Cargo.toml:
//!   embedded-io  = "0.6"
//!   embedded-hal = "1.0"

use embedded_hal::delay::DelayNs;
use embedded_io::{ Read, Write};


use super::{ParameterID, PayloadDecoder, SerialCmd, RadarError, RadarIoResult};

/// Driver generic over any TX/RX pair implementing the standard blocking I/O
/// traits. On esp-hal: `let (rx, tx) = uart.split();` — both halves implement
/// `Read`/`Write` when the driver is in `Blocking` mode.
///
/// Note: every call here busy-waits until the exchange completes. Under an
///  runtime this stalls the whole executor — use `MicrowaveRadarAsync`
/// there instead.
pub struct MicrowaveRadarIO<TX, RX, DELAY> {
    tx: TX,
    rx: RX,
    delay: DELAY,
}

impl<TX, RX, DELAY> MicrowaveRadarIO<TX, RX, DELAY>
where
TX: Write,
RX: Read,
DELAY: DelayNs,
{
    /// Delays supported as a parameter:
    /// `esp_hal::delay::Delay`
    /// any `embedded_hal::delay::DelayNs`
    #[allow(dead_code)]
    pub fn new(tx: TX, rx: RX, delay: DELAY) -> Self {
        Self { tx, rx, delay }
    }

    /// Sends one complete frame and waits until the FIFO has drained.
    /// `write_all` loops over partial writes for us — no manual retry needed.
    fn write_cmd(&mut self, data: &[u8]) -> RadarIoResult<(), TX, RX> {
        self.tx.write_all(data).map_err(RadarError::Tx)?;
        self.tx.flush().map_err(RadarError::Tx)
    }

    /// One byte of the reply. `read_exact` blocks until a byte actually arrives.
    pub fn next_byte(&mut self) -> RadarIoResult<u8, TX, RX> {
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
    ) -> RadarIoResult<Option<DECODER::Output>, TX, RX>
    where
    DECODER: PayloadDecoder,
    {
        self.write_cmd(&data.send)?;
        self.delay.delay_ns(data.delay_ns);

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
    ) -> RadarIoResult<(), TX, RX> {
        self.write_cmd(&data.send)?;
        self.delay.delay_ns(data.delay_ns);

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
    pub fn begin_config(&mut self) -> RadarIoResult<(), TX, RX> {
        self.send_cmd_and_check_ack_result(SerialCmd::begin_config())
    }

    /// Persists the configuration to the sensor's flash ("end/save config").
    pub fn end_save_config(&mut self) -> RadarIoResult<(), TX, RX> {
        self.send_cmd_and_check_ack_result(SerialCmd::end_save_config())
    }

    /// High-level helpers propagate the error with `?` instead of swallowing
    #[allow(dead_code)]
    pub fn set_report_mode_35byte_payload(&mut self) -> RadarIoResult<(), TX, RX> {
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode())
    }

    #[allow(dead_code)]
    pub fn set_report_debug_mode_1280byte_payload(&mut self) -> RadarIoResult<(), TX, RX>{
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode())
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
    ) -> RadarIoResult<Option<DECODER::Output>, TX, RX>
    where
    DECODER: PayloadDecoder,
    {
        self.send_cmd_and_get_result(SerialCmd::read_param_value(param_id), parser)
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
    pub fn set_range_delay_with_default_thresholds(&mut self, max_range: u8, delay_sec: u32){

        if self.begin_config().is_ok() || self.begin_config().is_ok() {

            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, max_range as f32));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, delay_sec as f32));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, ParameterID::TriggerThreshold00.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ParameterID::HoldThreshold00.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, ParameterID::TriggerThreshold01.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ParameterID::HoldThreshold01.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, ParameterID::TriggerThreshold02.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ParameterID::HoldThreshold02.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, ParameterID::TriggerThreshold03.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ParameterID::HoldThreshold03.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, ParameterID::TriggerThreshold04.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ParameterID::HoldThreshold04.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, ParameterID::TriggerThreshold05.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ParameterID::HoldThreshold05.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, ParameterID::TriggerThreshold06.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ParameterID::HoldThreshold06.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, ParameterID::TriggerThreshold07.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ParameterID::HoldThreshold07.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, ParameterID::TriggerThreshold08.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ParameterID::HoldThreshold08.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, ParameterID::TriggerThreshold09.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ParameterID::HoldThreshold09.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, ParameterID::TriggerThreshold10.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ParameterID::HoldThreshold10.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, ParameterID::TriggerThreshold11.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ParameterID::HoldThreshold11.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, ParameterID::TriggerThreshold12.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ParameterID::HoldThreshold12.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, ParameterID::TriggerThreshold13.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ParameterID::HoldThreshold13.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, ParameterID::TriggerThreshold14.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ParameterID::HoldThreshold14.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, ParameterID::TriggerThreshold15.default_value()));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ParameterID::HoldThreshold15.default_value()));


            // Do something after saving the configuration
            _ = self.end_save_config();
        }

        // self.set_report_mode_35byte_payload();

    }
    #[allow(dead_code)]
    pub fn set_params_value(&mut self, params_values: &[(ParameterID, f32)]){

        use super::parse_result::InitParser;
        let mut parser_params = super::parameter::ReadParam::new_parser();

        let mut value_for_param = |target: ParameterID| -> f32 {


            for (param_id, value) in params_values{

                if param_id == &target {
                    return *value;
                }
            }


            match self.get_param_value(target, &mut parser_params) {
                Ok(Some(v)) => v as f32,
                _ => target.default_value(),
            }
        };

        let range_gate = value_for_param(ParameterID::RangeGate);
        let absense_delay = value_for_param(ParameterID::AbsenseReportDelay);

        let tt00 = value_for_param(ParameterID::TriggerThreshold00);
        let ht00 = value_for_param(ParameterID::HoldThreshold00);

        let tt01 = value_for_param(ParameterID::TriggerThreshold01);
        let ht01 = value_for_param(ParameterID::HoldThreshold01);

        let tt02 = value_for_param(ParameterID::TriggerThreshold02);
        let ht02 = value_for_param(ParameterID::HoldThreshold02);

        let tt03 = value_for_param(ParameterID::TriggerThreshold03);
        let ht03 = value_for_param(ParameterID::HoldThreshold03);

        let tt04 = value_for_param(ParameterID::TriggerThreshold04);
        let ht04 = value_for_param(ParameterID::HoldThreshold04);

        let ht05 = value_for_param(ParameterID::HoldThreshold05);
        let tt05 = value_for_param(ParameterID::TriggerThreshold05);

        let tt06 = value_for_param(ParameterID::TriggerThreshold06);
        let ht06 = value_for_param(ParameterID::HoldThreshold06);

        let tt07 = value_for_param(ParameterID::TriggerThreshold07);
        let ht07 = value_for_param(ParameterID::HoldThreshold07);

        let tt08 = value_for_param(ParameterID::TriggerThreshold08);
        let ht08 = value_for_param(ParameterID::HoldThreshold08);

        let tt09 = value_for_param(ParameterID::TriggerThreshold09);
        let ht09 = value_for_param(ParameterID::HoldThreshold09);

        let tt10 = value_for_param(ParameterID::TriggerThreshold10);
        let ht10 = value_for_param(ParameterID::HoldThreshold10);

        let tt11 = value_for_param(ParameterID::TriggerThreshold11);
        let ht11 = value_for_param(ParameterID::HoldThreshold11);

        let tt12 = value_for_param(ParameterID::TriggerThreshold12);
        let ht12 = value_for_param(ParameterID::HoldThreshold12);

        let tt13 = value_for_param(ParameterID::TriggerThreshold13);
        let ht13 = value_for_param(ParameterID::HoldThreshold13);

        let tt14 = value_for_param(ParameterID::TriggerThreshold14);
        let ht14 = value_for_param(ParameterID::HoldThreshold14);

        let tt15 = value_for_param(ParameterID::TriggerThreshold15);
        let ht15 = value_for_param(ParameterID::HoldThreshold15);



        if self.begin_config().is_ok() || self.begin_config().is_ok(){

            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, range_gate));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, absense_delay));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, tt00));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ht00));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, tt01));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ht01));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, tt02));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ht02));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, tt03));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ht03));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, tt04));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ht04));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, tt05));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ht05));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, tt06));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ht06));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, tt07));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ht07));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, tt08));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ht08));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, tt09));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ht09));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, tt10));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ht10));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, tt11));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ht11));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, tt12));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ht12));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, tt13));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ht13));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, tt14));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ht14));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, tt15));
            _ = self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ht15));

            _ = self.end_save_config();
        }


    }
}