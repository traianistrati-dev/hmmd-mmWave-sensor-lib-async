//! Parameter read/write commands and the parser for read-parameter replies.

use super::{Parser, PayloadDecoder,SerialCmd, ParameterID, CommandID, SEND_HEADER,SEND_TAIL};

const CMD_HEADER: [u8; 4] = SEND_HEADER;
const CMD_TAIL:   [u8; 4] = SEND_TAIL;

const PAYLOAD_LEN: usize = 4;
const EXPECTED_CMD_ID: u16  = super::CommandID::ReadParamAck.as_u16();
const RESERVED_LEN: usize = 2;
const HAS_DATA_LENGHT: bool = true;


pub struct Decoder;

pub type DecoderType = Decoder;

type ParserType<'a> = Parser<'a, DecoderType, PAYLOAD_LEN, RESERVED_LEN,EXPECTED_CMD_ID, HAS_DATA_LENGHT>;

impl PayloadDecoder for DecoderType {
    type Output = u32;
    fn decode(&self, payload: &[u8]) -> Self::Output {
        u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]])
    }
}



pub struct ReadParam;


impl<'a> super::parse_result::InitParser<'a,DecoderType, PAYLOAD_LEN,  RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT> for ReadParam{

     fn new_parser() -> ParserType<'a>{

        ParserType::new(
            &CMD_HEADER,
            &CMD_TAIL,
            Some(DecoderType{})
        )
    }
}







/// Frame builder for reading a parameter value.
///
/// Reading a parameter requires a [`ReadParam`] parser instance to decode the reply.
///
/// # Example
///
/// ```ignore
/// let delay_micro_seconds_fn = |ms: u32| {
///     cortex_m::asm::delay(ms.saturating_mul(&clocks.sysclk().to_Hz() / 1_000_000));
/// };
///
/// let usart1_tx_write_fn = |data: &[u8]| {
///     for &b in data {
///         nb::block!(usart1_tx.write(b)).ok();
///     }
///     usart1_tx.flush().unwrap_or_default();
/// };
///
/// let usart1_rx_read_fn = || -> Option<u8> {
///     usart1_rx.read().ok()
/// };
///
/// let mut radar = hmmd_mmwave_sensor::MicrowaveRadar::new(
///     delay_micro_seconds_fn, usart1_tx_write_fn, usart1_rx_read_fn,
/// );
///
/// let mut parser_params = hmmd_mmwave_sensor::parameter::ReadParam::new_parser();
///
/// let range: Option<u32> = radar.get_param_value(
///     hmmd_mmwave_sensor::data::ParameterID::RangeGate, &mut parser_params);
/// let delay: Option<u32> = radar.get_param_value(
///     hmmd_mmwave_sensor::data::ParameterID::AbsenseReportDelay, &mut parser_params);
/// let tt_00: Option<u32> = radar.get_param_value(
///     hmmd_mmwave_sensor::data::ParameterID::TriggerThreshold00, &mut parser_params);
/// let ht_00: Option<u32> = radar.get_param_value(
///     hmmd_mmwave_sensor::data::ParameterID::HoldThreshold00, &mut parser_params);
/// ```
//send FD FC FB FA 04 00 08 00 01 00 04 03 02 01
//result ACK FD FC FB FA 08 00 08 01 00 00  0F 00 00 00  04 03 02 01
impl SerialCmd<14,0>{


    pub fn read_param_value(param_id:ParameterID) -> Self{

        let cmd_id_2b = CommandID::ReadParam.get_bytes();
        let param_id_2b = param_id.get_bytes();

        Self {
            send: [
                SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                0x04, 0x00,//data lenght
                cmd_id_2b[0], cmd_id_2b[1],
                param_id_2b[0],param_id_2b[1],
                SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            delay_us: 150,
            result_payload_ack:[]
        }

    }
}


/// Frame builder for writing a parameter value.
//send FD FC FB FA 08 00 07 00 01 00 02 00 00 00 04 03 02 01
//result ACK FD FC FB FA_ 04 00 _07 01_ 00 00 04 03 02 01
impl SerialCmd<18,4>{


    /// Builds the "write parameter" command that sets `param_id` to `param_value`.
    ///
    /// [`RangeGate`](ParameterID::RangeGate) and [`AbsenseReportDelay`](ParameterID::AbsenseReportDelay) are
    /// written as a raw integer; trigger/hold thresholds are encoded from dB via
    /// [`encode_threshold_value_to_le_bytes`](crate::encode_threshold_value_to_le_bytes).
    ///
    /// Each value is range-checked and falls back to
    /// [`default_value`](ParameterID::default_value) when out of range:
    /// `RangeGate` must be `< 16`, `AbsenseReportDelay` in `1..=99_999_999`, and
    /// thresholds in `1.0..=90.0` dB.
    pub fn set_param_value(param_id:ParameterID, param_value:f32) -> Self{

        let param_value_4b =  match &param_id {

            ParameterID::RangeGate => {

                let value:u32 = param_value as u32;
                if value < 16 {
                    value.to_le_bytes()
                }else {
                    (ParameterID::RangeGate.default_value() as u32).to_le_bytes()
                }
            },
            ParameterID::AbsenseReportDelay => {

                let value:u32 = param_value as u32;
                if 0 < value && value <= 99_999_999 {
                    value.to_le_bytes()
                }else {
                    (ParameterID::AbsenseReportDelay.default_value() as u32).to_le_bytes()
                }
            },
            param => {
                if (1.0..=90.0).contains(&param_value){
                    super::encode_threshold_value_to_le_bytes(param_value)
                }else{
                    super::encode_threshold_value_to_le_bytes(param.default_value())
                }
            }

        };

        let cmd_id_2b = CommandID::WriteParam.get_bytes();
        let cmd_id_ack_2b = CommandID::WriteParamAck.get_bytes();

        let param_id_2b = param_id.get_bytes();

        Self {
            send: [
                SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                0x08, 0x00,//data lenght
                cmd_id_2b[0], cmd_id_2b[1],
                param_id_2b[0],param_id_2b[1],
                param_value_4b[0],param_value_4b[1],param_value_4b[2],param_value_4b[3],
                SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            result_payload_ack:[
                //  SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                //0x04, 0x00,//data lenght
                cmd_id_ack_2b[0], cmd_id_ack_2b[1],
                0x00, 0x00,
                // SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            delay_us: 500,

        }
    }
}