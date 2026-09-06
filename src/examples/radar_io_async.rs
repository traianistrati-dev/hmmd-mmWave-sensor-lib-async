use crate::ParameterID;



async fn main(){


    /*USART 115200 baud, data bits 8, Parity None, Stop bit 1*/
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::_160MHz));
    let gpio5_usart1_rx = peripherals.GPIO5; // USART1  RX
    let gpio4_usart1_tx = peripherals.GPIO4; // USART1  TX
    let mut _uart1 = crate::uart1::init_async(peripherals.UART1, gpio5_usart1_rx, gpio4_usart1_tx);

    let (_tx, _rx) =_uart1.split();

    //------------------------------------------------------------------
    //radar instance
    use crate::parse_result::InitParser;
    let mut radar = crate::MicrowaveRadarIOAsync::new(_rx, _tx, embassy_time::Delay);


    //set parameters values
    radar.set_params_value(&[
            (ParameterID::RangeGate, 1.0),
            (ParameterID::AbsenseReportDelay, 5.0),
            // (ParameterID::HoldThreshold00, ParameterID::HoldThreshold00.default_value())
            //(ParameterID::HoldThreshold15, 20.0)
    ]);



    //Read parameters values

    let mut parser_params = crate::parameter::ReadParam::new_parser();


    if radar.begin_config().await.is_ok() {

        match radar.get_param_value(ParameterID::RangeGate, &mut parser_params).await
        {
            Ok(Some(radar_range_gate_val)) => {
            }
            _ => {}
        }

        //----------------------

        match radar.get_param_value(ParameterID::AbsenseReportDelay, &mut parser_params).await
        {
            Ok(Some(radar_delay_gate_val)) => { }
            _ => {}
        }


        //----------------------

        use crate::parse_result::decode_threschold_value;

        if let Ok(Some(radar_tt_00_val)) = radar.get_param_value(ParameterID::TriggerThreshold00, &mut parser_params).await {

            let tt00_values: f32 = decode_threschold_value(radar_tt_00_val);

        }

        if let Ok(Some(radar_ht_00_val)) = radar.get_param_value(ParameterID::HoldThreshold00, &mut parser_params).await {

            let ht00_values: f32 = decode_threschold_value(radar_ht_00_val);
        }

        _ = radar.end_save_config().await;
    }


    // set as report mode,  data from sensor as 45 byte frame
    // required before reading data
    radar.set_report_mode_35byte_payload().await;
    //
    let mut parser = crate::report_normal_mode::HmmdFrame::new_parser();

    loop{


        if let Ok(b) = radar.next_byte().await {

            if parser.feed(b)
            && let Some(frame) = parser.decode_payload()
            && frame.present {
                //frame.distance_cm

                // let energy_gate_0 = frame.energy[0];
                // //...
                // let energy_gate_15 = frame.energy[15];

            }
        }


    }



    // set as report debug mode,  data from sensor as 1288 byte frame
    // required before reading data
    radar.set_report_debug_mode_1280byte_payload().await;
    //
    let mut rdmap_parser = crate::report_debug_mode::HmmdRdmapFrame::new_parser();

    loop{


        if let Ok(b) = radar.next_byte().await {

            if parser.feed(b)
            && let Some(frame) = rdmap_parser.decode_payload()
            {

                let dople1_energy_gate0 = frame.rdmap[0][0];
                //...
                let dople20_energy_gate15 = frame.rdmap[19][15];

            }
        }
    }

}