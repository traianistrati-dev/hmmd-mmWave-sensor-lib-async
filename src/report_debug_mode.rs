//! Debug reporting mode: the range-Doppler map (RDMAP) frame and its command.
use super::{Parser, PayloadDecoder, CommandID};

const CMD_HEADER: [u8; 4] = [0xAA, 0xBF, 0x10, 0x14];
const CMD_TAIL: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];

const PAYLOAD_LEN: usize = 1280;

const COMMAND_ID: CommandID = CommandID::ReportMode;

const EXPECTED_CMD_ID: u16 = COMMAND_ID.as_u16();
const RESERVED_LEN: usize = 0;
const HAS_DATA_LENGHT_BYTES: bool = false;

pub struct Decoder;

type DecoderType = Decoder;

type ParserType<'a> = Parser<'a, DecoderType, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>;


impl PayloadDecoder for DecoderType {
    type Output = HmmdRdmapFrame;
    fn decode(&self, payload: &[u8]) -> Self::Output {
        if payload.len() != PAYLOAD_LEN {
            return HmmdRdmapFrame {
                rdmap: [[0u32; 16]; 20]
            };
        }

        let mut rdmap = [[0u32; 16]; 20];

        let mut index = 0;

        for doppler in &mut rdmap {

            for gate in doppler.iter_mut().take(16) {

                *gate = u32::from_le_bytes([
                        payload[index],
                        payload[index + 1],
                        payload[index + 2],
                        payload[index + 3],
                ]);

                index += 4;
            }
        }

        Self::Output {
            rdmap
        }
    }
}

/// A range-Doppler map frame (debug mode): a 20 (Doppler) × 16 (range gate)
/// matrix where each cell is the squared amplitude as a `u32`.
pub struct HmmdRdmapFrame {
    pub rdmap: [[u32; 16]; 20],
}


impl<'a> super::parse_result::InitParser<'a, DecoderType, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES> for HmmdRdmapFrame{

    fn new_parser() -> ParserType<'a>{

        ParserType::new(
            &CMD_HEADER,
            &CMD_TAIL,
            Some(DecoderType{})
        )
    }
}



/*
impl <'a>PayloadDecoder<'a, PAYLOAD_LEN,  RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES, HmmdRdmapFrame> for HmmdRdmapFrame {

    /// Builds a parser configured for the 1280-byte RDMAP frame.
    fn new_parser(&self) -> ParserType<'a> {
        ParserType::new(&CMD_HEADER, &CMD_TAIL)
    }

    /// Decodes the 1280-byte payload (320 little-endian `u32` values) into the
    /// 20×16 range-Doppler matrix. Returns a zero-filled frame if the payload
    /// length is unexpected.
    fn decode(&self, payload:&[u8]) -> Self{

        if payload.len() != PAYLOAD_LEN {
            return  Self {
                rdmap:[[0u32; 16]; 20]
            };
        }

        let mut rdmap = [[0u32; 16]; 20];

        let mut index = 0;

        for doppler in &mut rdmap {

            for gate in doppler.iter_mut().take(16) {

                *gate = u32::from_le_bytes([
                        payload[index],
                        payload[index + 1],
                        payload[index + 2],
                        payload[index + 3],
                ]);

                index += 4;
            }
        }

        Self {
            rdmap
        }

    }

}
*/

use super::{SerialCmd, SEND_HEADER, SEND_TAIL};

/// Frame builder for switching the sensor into debug mode.
//send FD FC FB FA 08 00 12 00 00 00 00 00 00 00 04 03 02 01
//result: 1280
/*
RDMAP: 20(Dopple)*16（number of range gate）*4（square of the amplitude）
320 values * 4bytes as Little Endian 01 00 00 00 -> 00 00 00 01 1u32
Dopple 0
g0-g15
....

AA BF 
10 --16 gates
14 --20 dopplers
7D 00 00 00 31 00 00 00 04 00 00 00 02 00 00 00 01 00 00 00 08 00 00 00 05 00 00 00 02 00 00 00 08 00 00 00 05 00 00 00 01 00 00 00 05 00 00 00 0D 00 00 00 05 00 00 00 05 00 00 00 09 00 00 00 
DA 00 00 00 3A 00 00 00 08 00 00 00 0D 00 00 00 05 00 00 00 05 00 00 00 05 00 00 00 05 00 00 00 08 00 00 00 0A 00 00 00 02 00 00 00 05 00 00 00 19 00 00 00 05 00 00 00 08 00 00 00 05 00 00 00 
5D 01 00 00 7A 00 00 00 0D 00 00 00 04 00 00 00 0A 00 00 00 01 00 00 00 05 00 00 00 02 00 00 00 05 00 00 00 05 00 00 00 05 00 00 00 14 00 00 00 05 00 00 00 0D 00 00 00 05 00 00 00 05 00 00 00 
04 00 00 00 09 00 00 00 08 00 00 00 08 00 00 00 19 00 00 00 09 00 00 00 04 00 00 00 08 00 00 00 0A 00 00 00 02 00 00 00 05 00 00 00 04 00 00 00 09 00 00 00 01 00 00 00 0A 00 00 00 09 00 00 00 
C8 00 00 00 41 00 00 00 04 00 00 00 01 00 00 00 09 00 00 00 08 00 00 00 05 00 00 00 14 00 00 00 05 00 00 00 01 00 00 00 05 00 00 00 01 00 00 00 01 00 00 00 05 00 00 00 0A 00 00 00 02 00 00 00 
88 00 00 00 3A 00 00 00 05 00 00 00 02 00 00 00 04 00 00 00 05 00 00 00 04 00 00 00 01 00 00 00 08 00 00 00 05 00 00 00 05 00 00 00 04 00 00 00 0A 00 00 00 01 00 00 00 04 00 00 00 05 00 00 00 
5A 00 00 00 24 00 00 00 08 00 00 00 01 00 00 00 02 00 00 00 04 00 00 00 05 00 00 00 04 00 00 00 10 00 00 00 04 00 00 00 04 00 00 00 00 00 00 00 05 00 00 00 04 00 00 00 02 00 00 00 04 00 00 00 
25 01 00 00 91 00 00 00 22 00 00 00 20 00 00 00 00 00 00 00 05 00 00 00 02 00 00 00 08 00 00 00 05 00 00 00 04 00 00 00 0A 00 00 00 05 00 00 00 00 00 00 00 05 00 00 00 02 00 00 00 02 00 00 00 
09 01 00 00 75 00 00 00 14 00 00 00 0A 00 00 00 01 00 00 00 0A 00 00 00 0A 00 00 00 02 00 00 00 08 00 00 00 01 00 00 00 02 00 00 00 05 00 00 00 02 00 00 00 04 00 00 00 02 00 00 00 08 00 00 00 
09 C2 00 00 39 73 00 00 02 08 00 00 28 00 00 00 08 00 00 00 12 00 00 00 02 00 00 00 08 00 00 00 04 00 00 00 04 00 00 00 09 00 00 00 05 00 00 00 0D 00 00 00 01 00 00 00 01 00 00 00 02 00 00 00 
34 93 03 00 F4 77 02 00 50 29 00 00 C1 00 00 00 19 00 00 00 24 00 00 00 20 00 00 00 0A 00 00 00 01 00 00 00 24 00 00 00 08 00 00 00 28 00 00 00 19 00 00 00 09 00 00 00 14 00 00 00 0A 00 00 00 
D0 1A 01 00 BD BF 00 00 0A 0A 00 00 41 00 00 00 24 00 00 00 05 00 00 00 24 00 00 00 1D 00 00 00 05 00 00 00 1A 00 00 00 0A 00 00 00 12 00 00 00 0A 00 00 00 08 00 00 00 05 00 00 00 05 00 00 00 
64 09 00 00 19 04 00 00 12 00 00 00 04 00 00 00 05 00 00 00 04 00 00 00 25 00 00 00 14 00 00 00 11 00 00 00 12 00 00 00 08 00 00 00 0D 00 00 00 0A 00 00 00 01 00 00 00 01 00 00 00 04 00 00 00 
DA 03 00 00 72 01 00 00 0D 00 00 00 08 00 00 00 05 00 00 00 00 00 00 00 02 00 00 00 08 00 00 00 02 00 00 00 08 00 00 00 0A 00 00 00 01 00 00 00 02 00 00 00 02 00 00 00 09 00 00 00 02 00 00 00 
22 01 00 00 52 00 00 00 0D 00 00 00 04 00 00 00 0D 00 00 00 02 00 00 00 01 00 00 00 02 00 00 00 05 00 00 00 08 00 00 00 02 00 00 00 04 00 00 00 00 00 00 00 05 00 00 00 05 00 00 00 05 00 00 00 
E9 00 00 00 7D 00 00 00 10 00 00 00 0A 00 00 00 0A 00 00 00 0D 00 00 00 0A 00 00 00 04 00 00 00 0D 00 00 00 02 00 00 00 08 00 00 00 08 00 00 00 0A 00 00 00 02 00 00 00 04 00 00 00 05 00 00 00 
52 01 00 00 6A 00 00 00 01 00 00 00 04 00 00 00 02 00 00 00 04 00 00 00 05 00 00 00 11 00 00 00 0A 00 00 00 0A 00 00 00 04 00 00 00 01 00 00 00 02 00 00 00 04 00 00 00 02 00 00 00 02 00 00 00 
64 01 00 00 4A 00 00 00 0A 00 00 00 12 00 00 00 0A 00 00 00 0A 00 00 00 14 00 00 00 09 00 00 00 0A 00 00 00 01 00 00 00 09 00 00 00 09 00 00 00 01 00 00 00 08 00 00 00 08 00 00 00 04 00 00 00 
B4 00 00 00 48 00 00 00 02 00 00 00 04 00 00 00 04 00 00 00 19 00 00 00 05 00 00 00 0D 00 00 00 02 00 00 00 0D 00 00 00 0A 00 00 00 05 00 00 00 04 00 00 00 0D 00 00 00 11 00 00 00 08 00 00 00 
8A 01 00 00 75 00 00 00 1A 00 00 00 05 00 00 00 00 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00 05 00 00 00 01 00 00 00 05 00 00 00 01 00 00 00 05 00 00 00 0A 00 00 00 04 00 00 00 05 00 00 00 
FD FC FB FA
*/
impl SerialCmd<18, 0>{
    pub fn set_report_debug_mode() -> Self{

        let cmd_id_2b = COMMAND_ID.get_bytes();


        Self {
            send: [
                SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                0x08, 0x00, //data lenght
                cmd_id_2b[0], cmd_id_2b[1],
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            result_payload_ack: [],
            delay_ns: 100,
        }
    }

}
