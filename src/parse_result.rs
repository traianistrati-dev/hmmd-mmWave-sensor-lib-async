//! Incremental, `no_std` byte-by-byte parser for HMMD protocol frames.

/// Internal state of the [`Parser`] state machine, tracking progress through a
/// frame: header, length, command id, reserved bytes, payload, tail.
enum State {
    Header(usize),
    Length(usize, u16),
    CmdId(usize, u16),
    ReservedBytes(usize),
    Payload(usize),
    Tail(usize),
}



pub trait PayloadDecoder {
    type Output;
    fn decode(&self, payload: &[u8]) -> Self::Output;
}


impl PayloadDecoder for () {
    type Output = ();

    fn decode(&self, _: &[u8]) {}
}


pub trait InitParser
<'a,
DECODER: PayloadDecoder,
const PAYLOAD_LEN: usize,
const RESERVED_LEN: usize,
const EXPECTED_CMD_ID: u16,
const HAS_DATA_LENGHT_BYTES: bool
>
{
    fn new_parser() -> Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>;
}


/// Incremental parser for HMMD frames, driven one byte at a time.
///
/// Feed received bytes with [`feed`](Parser::feed); it returns `true` once a
/// complete, valid frame has been assembled, at which point
/// [`payload`](Parser::payload) holds the useful bytes. Header/length/command-id/
/// tail are validated along the way and any mismatch resets the machine.
pub struct Parser
<'a,
DECODER,
const PAYLOAD_LEN: usize,
const RESERVED_LEN: usize,
const EXPECTED_CMD_ID: u16,
const HAS_DATA_LENGHT_BYTES: bool,
>
where DECODER: PayloadDecoder

{
    state: State,

    pub header: &'a [u8; 4],
    pub tail: &'a [u8; 4],

    pub length: u16,
    pub cmd_id: Option<u16>,
    pub reserved: [u8; RESERVED_LEN],
    pub payload: [u8; PAYLOAD_LEN],
    result_decoder: Option<DECODER>,
}

impl<'a,
DECODER,
const PAYLOAD_LEN: usize,
const RESERVED_LEN: usize,
const EXPECTED_CMD_ID: u16,
const HAS_DATA_LENGHT_BYTES: bool
>
Parser<'a, DECODER, PAYLOAD_LEN, RESERVED_LEN, EXPECTED_CMD_ID, HAS_DATA_LENGHT_BYTES>
where DECODER: PayloadDecoder
{
    pub const fn new(header: &'a[u8; 4], tail: &'a [u8; 4], result_decoder: Option<DECODER>) -> Self {
        Self {
            state: State::Header(0),
            header,
            tail,
            length: 0,
            cmd_id: None,
            reserved: [0u8; RESERVED_LEN],
            payload: [0u8; PAYLOAD_LEN],
            result_decoder
        }
    }



    pub fn decode_payload(&self) -> Option<DECODER::Output> {
        if let Some(decoder) = &self.result_decoder {
            return Some(decoder.decode(&self.payload));
        }
        None
    }

    #[inline]
    const fn has_cmd_id() -> bool {
        EXPECTED_CMD_ID != super::CommandID::None.as_u16()
    }
    pub fn clear(&mut self) {
        self.reset();
    }

    fn reset(&mut self) {
        self.state = State::Header(0);
        self.length = 0;
        self.cmd_id = None;
    }

    fn after_length_state() -> State {
        if Self::has_cmd_id() {
            State::CmdId(0, 0)
        } else if RESERVED_LEN > 0 {
            State::ReservedBytes(0)
        } else {
            State::Payload(0)
        }
    }

    fn after_cmd_state() -> State {
        if RESERVED_LEN > 0 {
            State::ReservedBytes(0)
        } else {
            State::Payload(0)
        }
    }

    /// Consumes one received byte, advancing the state machine.
    ///
    /// Returns `true` exactly when a complete, valid frame has just been
    /// finalized (the [`payload`](Parser::payload) is then available);
    /// otherwise returns `false`. When `HAS_DATA_LENGHT_BYTES` is `false`, the parser
    /// skips the length field and goes straight from the header to the next
    /// stage.
    pub fn feed(&mut self, b: u8) -> bool {
        match self.state {
            State::Header(n) => {
                if b == self.header[n] {
                    self.state = if n == 3 {
                        if HAS_DATA_LENGHT_BYTES {
                            State::Length(0, 0)
                        }else{
                            Self::after_length_state()
                        }
                    } else {
                        State::Header(n + 1)
                    };
                } else {
                    self.state = if b == self.header[0] {
                        State::Header(1)
                    } else {
                        State::Header(0)
                    };
                }
                false
            }

            // LENGTH = little-endian
            State::Length(n, acc) => {

                let acc = acc | ((b as u16) << (n * 8));

                if n == 1 {
                    self.length = acc;

                    let expected_len =
                    PAYLOAD_LEN as u16
                    + RESERVED_LEN as u16
                    + if Self::has_cmd_id() { 2 } else { 0 };

                    if acc == expected_len {
                        self.state = Self::after_length_state();
                    } else {
                        self.reset();
                    }
                } else {
                    self.state = State::Length(1, acc);
                }

                false
            }

            // CMD_ID = big-endian
            // bytes 08 01 => 0x0801
            State::CmdId(n, acc) => {
                let acc = if n == 0 {
                    (b as u16) << 8
                } else {
                    acc | (b as u16)
                };

                if n == 1 {
                    self.cmd_id = Some(acc);

                    if acc != EXPECTED_CMD_ID {
                        self.reset();
                        return false;
                    }

                    self.state = Self::after_cmd_state();
                } else {
                    self.state = State::CmdId(1, acc);
                }

                false
            }

            State::ReservedBytes(n) => {
                self.reserved[n] = b;
                self.state = if n + 1 == RESERVED_LEN {
                    State::Payload(0)
                } else {
                    State::ReservedBytes(n + 1)
                };
                false
            }

            State::Payload(n) => {
                self.payload[n] = b;


                self.state = if n + 1 == PAYLOAD_LEN {
                    State::Tail(0)
                } else {
                    State::Payload(n + 1)
                };

                false
            }

            State::Tail(n) => {
                if b == self.tail[n] {
                    if n == 3 {
                        self.reset();
                        true
                    } else {
                        self.state = State::Tail(n + 1);
                        false
                    }
                } else {
                    self.reset();
                    false
                }
            }
        }
    }
}

/// Decodes a raw threshold value back into dB (`10 * log10(value)`).
pub fn decode_threschold_value(value: u32) -> f32 {
    if value == 0 {
        return 0.0;
    }

    10.0 * libm::log10f(value as f32)
}