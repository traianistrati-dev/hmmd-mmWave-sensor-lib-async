//! High-level driver ([`MicrowaveRadar`]) and the HAL-agnostic I/O traits.

use super::{ParameterID, SerialCmd, PayloadDecoder};

/// High-level driver for the HMMD mmWave sensor.
///
/// Generic over the three I/O traits ([`Delay`], [`UsartTx`], [`UsartRx`]),
/// so it stays independent of any concrete HAL. Construct it with (MicrowaveRadar::new).
pub struct MicrowaveRadar<DELAY: Delay, TX: UsartTx, RX: UsartRx>{

    delay: DELAY,
    tx: TX,
    rx: RX,

}



/// Transmits bytes over the sensor's USART link.
///
/// Blanket-implemented for any `FnMut(&[u8])`, so a closure can be passed directly.
pub trait UsartTx {
    async fn write_bytes(&mut self, data: &[u8]);
}

impl<F> UsartTx for F where F: AsyncFnMut(&[u8]),
{
    async fn write_bytes(&mut self, data: &[u8]){
        self(data).await;
    }
}

/// Receives a single byte from the sensor's USART link.
///
/// Blanket-implemented for any `FnMut() -> Result<u8, nb::Error>.
pub trait UsartRx {
    type Error;
    async fn read_byte(&mut self) -> Result<u8, Self::Error>;
    // fn read_byte(&mut self) -> Result<u8, nb::Error<Self::Error>>;
}

// impl<F, E> UsartRx for F where F: FnMut() -> Result<u8, nb::Error<E>>,
impl<F, E> UsartRx for F where F: AsyncFnMut() -> Result<u8, E>,
{
    type Error = E;
    // fn read_byte(&mut self)-> Result<u8, nb::Error<E>>{
    async fn read_byte(&mut self)-> Result<u8, Self::Error>{
        self().await
    }
}

/// Blocking delay used to pace command/response exchanges.
///
/// Blanket-implemented for any `Fn(u32)`. The unit of the argument is defined by
/// the implementation (the crate passes the value of `SerialCmd::delay_us`)
pub trait Delay {
    async fn delay_us(&self, ms: u32);
}

impl<F> Delay for F where F: AsyncFn(u32),
{
    async fn delay_us(&self, ms: u32) {
        self(ms);
    }
}

impl <DELAY: Delay, TX: UsartTx, RX: UsartRx> MicrowaveRadar<DELAY, TX, RX>
{

    /// Creates a driver from a delay function and the TX/RX I/O handles.
    pub fn new(delay_fn: DELAY, tx: TX, rx: RX) -> Self {
        Self { delay: delay_fn, tx, rx}
    }


    async fn next_byte(&mut self) -> Option<u8> {
        loop {
            match self.rx.read_byte().await {
                Ok(b) => return Some(b),
                _ => {}
                // Err(nb::Error::WouldBlock) => {}
                // Err(nb::Error::Other(_err)) => return None,
            }
        }
    }

    /// Blocks using the configured delay function for `ms` time units.
    pub fn delay_us(&self, ms: u32) {

        self.delay.delay_us(ms);
    }


    /// Configures the maximum range and delay, loads the default trigger/hold
    /// thresholds for all 16 distance gates, saves the configuration, and puts
    /// the sensor into report mode.
    ///
    /// All parameters are written between `begin_config` and `end_save_config`.
    ///
    /// **Note:** every parameter must be written before `end_save_config()` for
    /// the configuration to be persisted to the sensor's flash.
    pub async fn set_range_delay_with_default_thresholds(&mut self, max_range: u8, delay_sec: u32){

        // let break_point = 0;

        if
        (self.begin_config().await || self.begin_config().await) {

            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, max_range as f32));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, delay_sec as f32));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, ParameterID::TriggerThreshold00.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ParameterID::HoldThreshold00.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, ParameterID::TriggerThreshold01.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ParameterID::HoldThreshold01.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, ParameterID::TriggerThreshold02.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ParameterID::HoldThreshold02.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, ParameterID::TriggerThreshold03.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ParameterID::HoldThreshold03.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, ParameterID::TriggerThreshold04.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ParameterID::HoldThreshold04.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, ParameterID::TriggerThreshold05.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ParameterID::HoldThreshold05.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, ParameterID::TriggerThreshold06.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ParameterID::HoldThreshold06.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, ParameterID::TriggerThreshold07.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ParameterID::HoldThreshold07.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, ParameterID::TriggerThreshold08.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ParameterID::HoldThreshold08.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, ParameterID::TriggerThreshold09.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ParameterID::HoldThreshold09.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, ParameterID::TriggerThreshold10.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ParameterID::HoldThreshold10.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, ParameterID::TriggerThreshold11.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ParameterID::HoldThreshold11.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, ParameterID::TriggerThreshold12.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ParameterID::HoldThreshold12.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, ParameterID::TriggerThreshold13.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ParameterID::HoldThreshold13.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, ParameterID::TriggerThreshold14.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ParameterID::HoldThreshold14.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, ParameterID::TriggerThreshold15.default_value()));
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ParameterID::HoldThreshold15.default_value()));


            // Do something after saving the configuration
            self.end_save_config();
        }

        // self.set_report_mode_35byte_payload();

    }



    pub async fn set_params_value(&mut self, params_values: &[(ParameterID, f32)]){

        use super::parse_result::InitParser;
        let mut parser_params = super::parameter::ReadParam::new_parser();

        let mut value_for_param = async|target: ParameterID| -> f32 {


            for (param_id, value) in params_values{

                if param_id == &target {
                    return *value;
                }
            }


            self.get_param_value(target, &mut parser_params).await
            .map(|v| v as f32)
            .unwrap_or(target.default_value())
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



        if self.begin_config().await || self.begin_config().await{

            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, range_gate)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, absense_delay)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, tt00)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ht00)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, tt01)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ht01)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, tt02)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ht02)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, tt03)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ht03)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, tt04)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ht04)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, tt05)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ht05)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, tt06)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ht06)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, tt07)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ht07)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, tt08)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ht08)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, tt09)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ht09)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, tt10)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ht10)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, tt11)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ht11)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, tt12)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ht12)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, tt13)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ht13)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, tt14)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ht14)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, tt15)).await;
            self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ht15)).await;
            self.end_save_config().await;
        }


    }



    /// Reads the value of a single parameter from the sensor.
    ///
    /// `parser` must be created with (crate::parameter::ReadParam::new_parser).
    /// Returns `None` on timeout or an invalid reply.
    // pub fn get_param_value<const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16,const HAS_DATA_LENGHT_BYTES: bool>(
    // &mut self
    // ,param_id:ParameterID
    // ,parser:&mut super::Parser<PAYLOAD_LEN,RESERVED_LEN,EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>

    pub async fn get_param_value<'a, DECODER, const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16, const HAS_DATA_LENGHT_BYTES: bool>(
        &mut self
        , param_id: ParameterID
        //,param_parser: super::parameter::ReadParam
        , parser: &mut super::Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>
    ) -> Option<DECODER::Output> where DECODER: PayloadDecoder
    {

        self.send_cmd_and_get_result(
            SerialCmd::read_param_value(param_id)
            , parser
        ).await

    }

    /// Sends a command, then parses the reply with `parser` and decodes it with `decode`.
    ///
    /// Returns the decoded result, or `None` if no valid frame arrives before the
    /// internal idle timeout.
    pub async fn send_cmd_and_get_result<'a, DECODER, const S: usize, const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16, const HAS_DATA_LENGHT_BYTES: bool>(
        &mut self,
        data: SerialCmd<S, 0>,
        parser: &mut super::Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>,

    ) -> Option<DECODER::Output> where DECODER: PayloadDecoder
    {
        self.tx.write_bytes(&data.send);

        self.delay_us(data.delay_us);

        //parser.clear();

        while let Some(b) = self.next_byte().await {
            if parser.feed(b) {
                // return Some(decode(&parser.payload));
                return parser.decode_payload();
            }
        }


        None

    }



    /// Sends a command and checks whether the received ACK matches the expected
    /// payload.
    ///
    /// Returns `true` on a match — or immediately when the command defines no ACK
    /// payload (`result_payload_ack` empty); returns `false` on mismatch or timeout.
    pub async fn send_cmd_and_check_ack_result< const S: usize, const R: usize>(&mut self, data: SerialCmd<S, R>) -> bool{
        self.tx.write_bytes(&data.send);

        self.delay_us(data.delay_us);


        if data.result_payload_ack.is_empty() {
            return true;
        }
        //Parser<'a, DecoderType, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>;


        let mut parser = super::Parser::<(), R, 0, { super::CommandID::None.as_u16() }, true>::new(&super::SEND_HEADER, &super::SEND_TAIL, None);

        while let Some(b) = self.next_byte().await {
            if parser.feed(b) {
                for i in 0..R{
                    if data.result_payload_ack[i] != parser.payload[i] {
                        return false;
                    }
                }
                return true;
            }
        }

        false

    }


    /// Enters configuration mode (the "enable config" command).
    pub async fn begin_config(&mut self) -> bool{
        self.send_cmd_and_check_ack_result(SerialCmd::begin_config()).await
    }

    /// Saves the configuration to flash (the "end/save config" command).
    pub async fn end_save_config(&mut self) -> bool{
        self.send_cmd_and_check_ack_result(SerialCmd::end_save_config()).await
    }

    pub fn set_report_mode_35byte_payload(&mut self){
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode());
    }

    pub fn set_report_debug_mode_1280byte_payload(&mut self){
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode());
    }

}


