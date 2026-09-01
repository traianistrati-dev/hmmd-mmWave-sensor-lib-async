//! Builders for outgoing command frames ([`SerialCmd`]) and threshold encoding.

use super::{CommandID, SEND_HEADER, SEND_TAIL};


/// A ready-to-send serial command frame together with its expected ACK.
///
/// The const generics describe the frame's byte layout: `S` is the length of
/// the [`send`](SerialCmd::send) frame, and `R` is the length of the ACK
/// [`result_payload_ack`](SerialCmd::result_payload_ack) compared against the
/// sensor's response (`0` when the command returns no ACK payload).
pub struct SerialCmd<const S: usize, const R: usize>{

    pub send: [u8; S],
    pub result_payload_ack: [u8; R],
    pub delay_us: u32,

}


/// Frame builder for entering configuration mode.
impl SerialCmd<14, 8>{

    /// Builds the "enable config" command: enters configuration mode.
    ///
    /// Must be sent before writing any parameter values.
    /// Sent:   `FD FC FB FA 04 00 FF 00 02 00 04 03 02 01`.
    /// ACK:    `FD FC FB FA 08 00 FF 01 00 00 02 00 20 00 04 03 02 01`.
    pub fn begin_config() -> Self{

        let cmd_id_2b = CommandID::EnableConfig.get_bytes();
        let cmd_id_ack_2b = CommandID::EnableConfigAck.get_bytes();

        Self {
            send: [
                SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                0x04, 0x00, //data lenght
                cmd_id_2b[0], cmd_id_2b[1],
                0x02, 0x00,
                SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            result_payload_ack: [
                 //SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                //0x08, 0x00,//data lenght
                cmd_id_ack_2b[0], cmd_id_ack_2b[1],
                0x00, 0x00,
                0x02, 0x00, 0x20, 0x00,
                // SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            delay_us: 2000,
        }
    }
}


/// Frame builder for saving the configuration.
impl SerialCmd<12, 4>{
    /// Builds the "end/save config" command: persists the configuration to the
    /// sensor's flash. Sent after all parameter values have been written.
    ///
    /// Sent:   `FD FC FB FA 02 00 FE 00 04 03 02 01`.
    /// ACK:    `FD FC FB FA 04 00 FE 01 00 00 04 03 02 01`.
    pub fn end_save_config() -> Self{

        let cmd_id_2b = CommandID::EndSaveConfig.get_bytes();
        let cmd_id_ack_2b = CommandID::EndSaveConfigAck.get_bytes();


        Self {
            send: [
                SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                0x02, 0x00, //data lenght
                cmd_id_2b[0], cmd_id_2b[1],
                SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            result_payload_ack: [
                 //SEND_HEADER[0], SEND_HEADER[1], SEND_HEADER[2], SEND_HEADER[3],
                //0x04, 0x00,//data lenght
                cmd_id_ack_2b[0], cmd_id_ack_2b[1],
                0x00, 0x00,
                 //SEND_TAIL[0], SEND_TAIL[1], SEND_TAIL[2], SEND_TAIL[3],
            ],
            delay_us: 2000,
        }
    }

}



/// Encodes a threshold value (in dB) into 4 little-endian bytes.
///
/// Applies `10^(value / 10)` and truncates to `u32`.
pub fn encode_threshold_value_to_le_bytes(value: f32) -> [u8; 4] {
    if value == 0.0 {
        return [0x00, 0x00, 0x00, 0x00];
    }
    (libm::powf(10.0, value / 10.0) as u32).to_le_bytes()
}

