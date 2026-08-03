





use super::data::ParameterID;



fn main(){



    let (mut _tx1, mut _rx1) = /*USART 115200 baud, data bits 8, Parity None, Stop bit 1*/((),());

    //------------------------------------------------------------------
    // Required closure functions

    let delay_micro_seconds_fn = |us:u32|{
        // cortex_m::asm::delay(us.saturating_mul(&clocks.sysclk().to_Hz() / 1_000_000));
    };

    let usart1_tx_write_fn = |data: &[u8]| {

        // for &b in data {
        // nb::block!(_tx1.write(b)).ok();
        // }
        // _tx1.flush().unwrap_or_default();
    };

    let usart1_rx_read_fn = || -> Result<u8, nb::Error<()/*stm32f1xx_hal::serial::Error*/>> {

        // _rx1.read()
        Ok(0)
    };
    //------------------------------------------------------------------
    //radar instance
    let mut radar = super::MicrowaveRadar::new(delay_micro_seconds_fn, usart1_tx_write_fn, usart1_rx_read_fn);


	//set parameters values
    radar.set_params_value(&[
            (ParameterID::AbsenseReportDelay, 5.0),
            (ParameterID::RangeGate, 1.0),
            // (ParameterID::HoldThreshold00, ParameterID::HoldThreshold00.default_value())
            //(ParameterID::HoldThreshold15, 20.0)
    ]);



    //Read parameters values
    use super::parse_result::ParserResult;
    let mut parser_params = super::parameter::ReadParam::new_parser();

    let radar_range_gate_val: Option<u32> = radar.get_param_value(ParameterID::RangeGate ,&mut parser_params);

    let radar_delay_gate_val:Option<u32> = radar.get_param_value(ParameterID::AbsenseReportDelay,&mut parser_params);

    {
        let radar_tt_00_val:Option<u32> = radar.get_param_value(ParameterID::TriggerThreshold00, &mut parser_params);

        let radar_ht_00_val:Option<u32> = radar.get_param_value(ParameterID::HoldThreshold00, &mut parser_params );

        use super::parse_result::decode_threschold_value;
        let tt00_values:f32 = decode_threschold_value(radar_tt_00_val.unwrap_or_default());
        let ht00_values:f32 = decode_threschold_value(radar_ht_00_val.unwrap_or_default());
    }


    {
        // set as report mode,  data from sensor as 45 byte frame
        // required before reading data
        radar.set_report_mode_35byte_payload();
        //
        let mut parser = super::report_normal_mode::HmmdFrame::new_parser();


        loop{
            radar.read_byte(|b| {

                    if parser.feed(b) {
                        let frame = super::report_normal_mode::HmmdFrame::decode(&parser.payload);
                        if frame.present {

                        }
                    }

            });
        }
    }


    {
        // set as report debug mode,  data from sensor as 1288 byte frame
        // required before reading data
        radar.set_report_debug_mode_1280byte_payload();
        //
        let mut parser = super::report_debug_mode::HmmdRdmapFrame::new_parser();


        loop{
            radar.read_byte(|b| {

                    if parser.feed(b) {
                        let frame: crate::HmmdRdmapFrame = super::report_debug_mode::HmmdRdmapFrame::decode(&parser.payload);
                        let dople1_energy_gate0 = frame.rdmap[0][0];
                        //...
                        let dople20_energy_gate16 = frame.rdmap[19][15];
                    }

            });
        }
    }





}