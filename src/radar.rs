//! High-level driver ([`MicrowaveRadar`]) and the HAL-agnostic I/O traits.

use super::{ParameterID, SerialCmd, PayloadDecoder};

/// High-level driver for the HMMD mmWave sensor.
///
/// Generic over the three I/O traits ([`Delay`], [`UsartTx`], [`UsartRx`]),
/// so it stays independent of any concrete HAL. Construct it with (MicrowaveRadar::new).
pub struct MicrowaveRadar<DELAY:Delay,TX:UsartTx,RX:UsartRx>{

    delay: DELAY,
    tx:TX,
    rx:RX,

}




/// Transmits bytes over the sensor's USART link.
///
/// Blanket-implemented for any `FnMut(&[u8])`, so a closure can be passed directly.
pub trait UsartTx {
    fn write_bytes(&mut self, data: &[u8]);
}

impl<F> UsartTx for F where F: FnMut(&[u8]),
{
    fn write_bytes(&mut self, data: &[u8]){
        self(data);
    }
}

/// Receives a single byte from the sensor's USART link.
///
/// Blanket-implemented for any `FnMut() -> Result<u8, nb::Error>.
pub trait UsartRx {
    type Error;
    fn read_byte(&mut self) -> Result<u8, nb::Error<Self::Error>>;
}

impl<F,E> UsartRx for F where F: FnMut() -> Result<u8, nb::Error<E>>,
{
    type Error = E;
    fn read_byte(&mut self)-> Result<u8, nb::Error<E>>{
        self()
    }
}

/// Blocking delay used to pace command/response exchanges.
///
/// Blanket-implemented for any `Fn(u32)`. The unit of the argument is defined by
/// the implementation (the crate passes the value of `SerialCmd::delay_us`)
pub trait Delay {
    fn delay_us(&self, ms: u32);
}

impl<F> Delay for F where F: Fn(u32),
{
    fn delay_us(&self, ms: u32) {
        self(ms);
    }
}

impl <DELAY:Delay, TX:UsartTx,RX:UsartRx> MicrowaveRadar<DELAY,TX,RX>{

    /// Creates a driver from a delay function and the TX/RX I/O handles.
    pub fn new(delay_fn: DELAY, tx:TX,rx:RX) -> Self {
        Self { delay:delay_fn, tx, rx}
    }

    /// Configures the maximum range and delay, loads the default trigger/hold
    /// thresholds for all 16 distance gates, saves the configuration, and puts
    /// the sensor into report mode.
    ///
    /// All parameters are written between `begin_config` and `end_save_config`.
    ///
    /// **Note:** every parameter must be written before `end_save_config()` for
    /// the configuration to be persisted to the sensor's flash.
    pub fn set_range_delay_with_default_thresholds(&mut self, max_range:u8, delay_sec:u32){


        if self.begin_config() && self.begin_config()

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, max_range as f32))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay, delay_sec as f32))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00, ParameterID::TriggerThreshold00.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ParameterID::HoldThreshold00.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01, ParameterID::TriggerThreshold01.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ParameterID::HoldThreshold01.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02, ParameterID::TriggerThreshold02.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ParameterID::HoldThreshold02.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03, ParameterID::TriggerThreshold03.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ParameterID::HoldThreshold03.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04, ParameterID::TriggerThreshold04.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ParameterID::HoldThreshold04.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05, ParameterID::TriggerThreshold05.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ParameterID::HoldThreshold05.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06, ParameterID::TriggerThreshold06.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ParameterID::HoldThreshold06.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07, ParameterID::TriggerThreshold07.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ParameterID::HoldThreshold07.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08, ParameterID::TriggerThreshold08.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ParameterID::HoldThreshold08.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09, ParameterID::TriggerThreshold09.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ParameterID::HoldThreshold09.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10, ParameterID::TriggerThreshold10.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ParameterID::HoldThreshold10.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11, ParameterID::TriggerThreshold11.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ParameterID::HoldThreshold11.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12, ParameterID::TriggerThreshold12.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ParameterID::HoldThreshold12.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13, ParameterID::TriggerThreshold13.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ParameterID::HoldThreshold13.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14, ParameterID::TriggerThreshold14.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ParameterID::HoldThreshold14.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15, ParameterID::TriggerThreshold15.default_value()))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ParameterID::HoldThreshold15.default_value()))

        && self.end_save_config(){
            // Do something after saving the configuration
        }

        // self.set_report_mode_35byte_payload();

    }



    pub fn set_params_value(&mut self, params_values: &[(ParameterID, f32)] ){
		
		use super::parse_result::InitParser;
        let mut parser_params = super::parameter::ReadParam::new_parser();

        let mut value_for_param = |target: ParameterID| -> f32 {


            for (param_id, value) in params_values{

                if param_id == &target {
                    return  *value;
                }
            }


            self.get_param_value(target, &mut parser_params)
            .map(|v| v as f32)
            .unwrap_or(target.default_value())
        };

        let range_gate =  value_for_param( ParameterID::RangeGate);
        let absense_delay =  value_for_param( ParameterID::AbsenseReportDelay);

        let tt00 =  value_for_param( ParameterID::TriggerThreshold00);
        let ht00 =  value_for_param( ParameterID::HoldThreshold00);

        let tt01 =  value_for_param( ParameterID::TriggerThreshold01);
        let ht01 =  value_for_param( ParameterID::HoldThreshold01);

        let tt02 =  value_for_param( ParameterID::TriggerThreshold02);
        let ht02 =  value_for_param( ParameterID::HoldThreshold02);

        let tt03 =  value_for_param( ParameterID::TriggerThreshold03);
        let ht03 =  value_for_param( ParameterID::HoldThreshold03);

        let tt04 =  value_for_param( ParameterID::TriggerThreshold04);
        let ht04 =  value_for_param( ParameterID::HoldThreshold04);

        let ht05 =  value_for_param( ParameterID::HoldThreshold06);
        let tt05 =  value_for_param( ParameterID::TriggerThreshold05);

        let tt06 =  value_for_param( ParameterID::TriggerThreshold06);
        let ht06 =  value_for_param( ParameterID::HoldThreshold06);

        let tt07 =  value_for_param( ParameterID::TriggerThreshold07);
        let ht07 =  value_for_param( ParameterID::HoldThreshold07);

        let tt08 =  value_for_param( ParameterID::TriggerThreshold08);
        let ht08 =  value_for_param( ParameterID::HoldThreshold08);

        let tt09 =  value_for_param( ParameterID::TriggerThreshold09);
        let ht09 =  value_for_param( ParameterID::HoldThreshold09);

        let tt10 =  value_for_param( ParameterID::TriggerThreshold10);
        let ht10 =  value_for_param( ParameterID::HoldThreshold10);

        let tt11 =  value_for_param( ParameterID::TriggerThreshold11);
        let ht11 =  value_for_param( ParameterID::HoldThreshold11);

        let tt12 =  value_for_param( ParameterID::TriggerThreshold12);
        let ht12 =  value_for_param( ParameterID::HoldThreshold12);

        let tt13 =  value_for_param( ParameterID::TriggerThreshold13);
        let ht13 =  value_for_param( ParameterID::HoldThreshold13);

        let tt14 =  value_for_param(  ParameterID::TriggerThreshold14);
        let ht14 =  value_for_param(  ParameterID::HoldThreshold14);

        let tt15 =  value_for_param( ParameterID::TriggerThreshold15);
        let ht15 =  value_for_param( ParameterID::HoldThreshold15);



        if self.begin_config() && self.begin_config()

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::RangeGate, range_gate))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::AbsenseReportDelay,absense_delay))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold00,tt00 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold00, ht00))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold01,tt01 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold01, ht01))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold02,tt02 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold02, ht02))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold03,tt03 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold03, ht03))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold04,tt04 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold04, ht04))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold05,tt05 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold05, ht05))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold06,tt06 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold06, ht06))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold07,tt07 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold07, ht07))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold08,tt08 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold08, ht08))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold09,tt09 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold09, ht09))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold10,tt10 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold10, ht10))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold11,tt11 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold11, ht11))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold12,tt12 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold12, ht12))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold13,tt13 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold13, ht13))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold14,tt14 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold14, ht14))

        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::TriggerThreshold15,tt15 ))
        && self.send_cmd_and_check_ack_result(SerialCmd::set_param_value(ParameterID::HoldThreshold15, ht15))


        && self.end_save_config(){
            // Do something after saving the configuration
        }


    }



    /// Reads a single byte, if available, and hands it to `read_fn`.
    pub fn read_byte(&mut self,mut read_fn:impl FnMut( u8)){

        if let Some(b) = self.next_byte() {
            read_fn(b)
        }
    }

    fn next_byte(&mut self) -> Option<u8> {
        loop {
            match self.rx.read_byte() {
                Ok(b) => return Some(b),
                Err(nb::Error::WouldBlock) => {}
                Err(nb::Error::Other(_err)) => return None,
            }
        }
    }

    /// Blocks using the configured delay function for `ms` time units.
    pub fn delay_us(&self, ms:u32) {

        self.delay.delay_us(ms);
    }

    /// Reads the value of a single parameter from the sensor.
    ///
    /// `parser` must be created with (crate::parameter::ReadParam::new_parser).
    /// Returns `None` on timeout or an invalid reply.
    // pub fn get_param_value<const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16,const HAS_DATA_LENGHT: bool>(
    // &mut self
    // ,param_id:ParameterID
    // ,parser:&mut super::Parser<PAYLOAD_LEN,RESERVED_LEN,EXPECTED_CMD_ID, HAS_DATA_LENGHT>

    pub fn get_param_value<DECODER,const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16,const HAS_DATA_LENGHT: bool>(
        &mut self
        ,param_id:ParameterID
        //,param_parser: super::parameter::ReadParam
        ,parser:&mut super::Parser<DECODER,PAYLOAD_LEN,RESERVED_LEN,EXPECTED_CMD_ID, HAS_DATA_LENGHT>
    ) -> Option<DECODER::Output>  where DECODER:PayloadDecoder
    {

        self.send_cmd_and_get_result(
            SerialCmd::read_param_value(param_id)
            ,parser
        )

    }

    /// Sends a command, then parses the reply with `parser` and decodes it with `decode`.
    ///
    /// Returns the decoded result, or `None` if no valid frame arrives before the
    /// internal idle timeout.
    pub fn send_cmd_and_get_result<DECODER, const S:usize,const PAYLOAD_LEN: usize, const RESERVED_LEN: usize, const EXPECTED_CMD_ID: u16,const HAS_DATA_LENGHT: bool>(
        &mut self,
        data:SerialCmd<S,0>,
        parser: &mut super::Parser<DECODER, PAYLOAD_LEN,RESERVED_LEN,EXPECTED_CMD_ID, HAS_DATA_LENGHT>,

    ) -> Option<DECODER::Output> where DECODER:PayloadDecoder
    {
        self.tx.write_bytes(&data.send);

        self.delay_us(data.delay_us);

        parser.clear();

        while let Some(b) = self.next_byte() {
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
    pub fn send_cmd_and_check_ack_result< const S:usize, const R:usize>(&mut self, data:SerialCmd<S,R>) -> bool{
        self.tx.write_bytes(&data.send);

        self.delay_us(data.delay_us);


        if data.result_payload_ack.is_empty() {
            return true;
        }

        // type DECODER;

        let mut parser = super::Parser::<(), R, 0, { super::CommandID::None.as_u16() }, true>::new(&super::SEND_HEADER, &super::SEND_TAIL, None);

        while let Some(b) = self.next_byte() {
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
    pub fn begin_config(&mut self) -> bool{
        self.send_cmd_and_check_ack_result(SerialCmd::begin_config())
    }

    /// Saves the configuration to flash (the "end/save config" command).
    pub fn end_save_config(&mut self) -> bool{
        self.send_cmd_and_check_ack_result(SerialCmd::end_save_config())
    }

    pub fn set_report_mode_35byte_payload(&mut self){
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_mode());
    }

    pub fn set_report_debug_mode_1280byte_payload(&mut self){
        self.send_cmd_and_check_ack_result(SerialCmd::set_report_debug_mode());
    }

}


