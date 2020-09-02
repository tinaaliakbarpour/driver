//#include "recorder.hpp"
//#include <pax_reload.hpp>
//#include <string>

//int main(int argc, char* argv[])
//{
//    (void)argc;
//    (void) argv;

//        mb_container_type tester;
//        std::vector<boost::shared_ptr<pax::transport::sph::recv_packet_streamer> > streamers=pax_init(tester,8);

// //       tester.iface->poke32(U2_REG_SR_ADDR(104), 1<<0);// to_wb
////        tester.fifo_ctrl->poke32(U2_REG_SR_ADDR(104), 1<<0);// sfc_without_wb

////        while(1) std::cout << tester.fifo_ctrl->peek32(U2_REG_SR_ADDR(24)) << std::endl;

////        std::cout <<1;

//        //tester.filter_bank[4]->set_filter_path_simulator(1000e6, "RX", false);
//        //std::cout << "ok!";


////        while(1){
////            for(int i = 0; i < 3000; i++)
////                for(int j = 2; j < 4; j++)
////                    tester.ad_9361[j]->tune("RX",(50+i)*1e6);
////        }
//        /*pax::flash_vec_t x = tester.iface->read_uid_flash();
//        std::cout << "%%%%%%%%" << std::endl;

//        for(int i = 0; i < x.size(); i++)
//            std::cout << x[i] << std::endl;*/

//        //tester.iface->erase_flash(0x0,10);
//        //tester.iface->write_flash(0x5,582);
//        //pax::flash_vec_t uid = tester.iface->read_uid_flash();//PH


//        //pax::pax_reload::sptr reloader = pax::pax_reload::make(tester.iface);
//        //reloader->burn_fpga_image(0,"/home/parto/Desktop/pax/PAX8/Pax8V7/PAX8_V7/PAX8_V7_fw_UNLOCK_.bit");

//        //tester.iface->_bpi_write_reg(5,666);
//        //std::cout << "****" << tester.iface->_bpi_read_reg(5);

//        /*{

//            tester.iface->cfi_read_all_device_ident();
//            tester.iface->cfi_queries();
//            tester.iface->cfi_unlock_and_erase_block(0);


//            tester.iface->_bpi_write_reg(5,666);
//            std::cout << "************" << tester.iface->_bpi_read_reg(0xE01004E) << std::endl;


//            std::cout << "begin: " << std::endl;
//            clock_t begin = clock();
//            for(int i=0; i<8; i++) {
//                tester.iface->cfi_data_write_32bits(0x00000000 + 4*i, 0x000c0000 + i);
//            }
//            clock_t end = clock();
//            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//            std::cout << "write time: " << elapsed_secs<<std::endl;

//            begin = clock();
//            for(int i=0; i<8; i++) {
//                uint32_t t1 = tester.iface->cfi_read_16_bits(0x00000000 +4*i);
//                uint32_t t2 = tester.iface->cfi_read_16_bits(0x00000000 +4*i+2);
//                uint32_t res = t2 + (t1 << 16);
//                if(res != (0x000c0000 + i))
//                    std::cout << t1 << "    ___   " << t2 << std::endl;
//            }
//            end = clock();
//            std::cout << "end: " << std::endl;
//            elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//            std::cout << "read time: " << elapsed_secs<<std::endl;
//            uint32_t time = tester.iface->cfi_read_32_bits(32*1024*4);
//            std::cout<<time<<std::endl;

//        }*/













////       tester.iface->poke32(U2_REG_SR_ADDR(1022),0);
////        tester.iface->poke32(U2_REG_SR_ADDR(1023),65535);

////        while(true){
////        tester.iface->poke32(U2_REG_SR_ADDR(101),0x0);
////        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
////        tester.iface->poke32(U2_REG_SR_ADDR(101),0x1);
////        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

////}


////        while(1){
////            std::cout << tester.iface->peek32(U2_REG_SR_ADDR(101)) << std::endl;
////            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
////        }
//        //tester.sync->PAX8V7_calibration();
//        //      tester.iface->poke32(U2_REG_SR_ADDR(100),8); //GO TO 12V


//        //        typedef enum {low_pass_40MHz=0,low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz,low_pass_2200MHz,low_pass_6000MHz} FILTER_PATH_virtex;



//        int which_9364 = 4;
//        long int freq = 5800e6;
//        int gain = 50;


//        for (uint32_t i=0; i<(tester.N_AD9361 + tester.N_AD9364); i++){
//            if(i != which_9364){
//            tester.ad_9361[i]->set_active_chains(false,false,false,false);
//            tester.ad_9361[i]->tune("TX",freq,true);
//            tester.ad_9361[i]->set_gain("TX1",0);
//            tester.ad_9361[i]->output_digital_test_tone(false);
//            }
//        }

//        tester.ad_9361[which_9364]->set_active_chains(true,false,false,false);
//        tester.ad_9361[which_9364]->tune("TX",freq,true);
//        tester.ad_9361[which_9364]->set_gain("TX1",gain);
//        tester.ad_9361[which_9364]->output_digital_test_tone(true);


//        // enable clock from ad93611
//        tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xff);


//        std::cout<<"1";

//        while(true){
//            /*tester.filter_bank[0]->set_filter_path_simulator(900e6,"TX",false);
//            tester.ad_9361[0]->tune("TX",50e6,false);
//            tester.ad_9361[0]->tune("TX",500e6,false);
//            tester.ad_9361[0]->tune("TX",1000e6,false);
//            tester.filter_bank[0]->set_filter_path_simulator(2200e6,"TX",false);
//            tester.ad_9361[0]->tune("TX",50e6,false);
//            tester.ad_9361[0]->tune("TX",1000e6,false);
//            tester.ad_9361[0]->tune("TX",2400e6,false);

//            tester.filter_bank[2]->set_filter_path_simulator(100e6,"TX",false);
//            tester.ad_9361[2]->tune("TX",50e6,false);
//            tester.ad_9361[2]->tune("TX",90e6,false);
//            tester.ad_9361[2]->tune("TX",130e6,false);
//            tester.filter_bank[2]->set_filter_path_simulator(200e6,"TX",false);
//            tester.ad_9361[2]->tune("TX",50e6,false);
//            tester.ad_9361[2]->tune("TX",100e6,false);
//            tester.ad_9361[2]->tune("TX",264e6,false);
//            tester.filter_bank[2]->set_filter_path_simulator(300e6,"TX",false);
//            tester.ad_9361[2]->tune("TX",50e6,false);
//            tester.ad_9361[2]->tune("TX",200e6,false);
//            tester.ad_9361[2]->tune("TX",470e6,false);
//            tester.filter_bank[2]->set_filter_path_simulator(500e6,"TX",false);
//            tester.ad_9361[2]->tune("TX",50e6,false);
//            tester.ad_9361[2]->tune("TX",500e6,false);
//            tester.ad_9361[2]->tune("TX",1000e6,false);



//            tester.filter_bank[4]->set_filter_path_simulator(900e6,"TX",false);
//            tester.ad_9361[4]->tune("TX",50e6,false);
//            tester.ad_9361[4]->tune("TX",500e6,false);
//            tester.ad_9361[4]->tune("TX",1000e6,false);
//            tester.filter_bank[4]->set_filter_path_simulator(2200e6,"TX",false);
//            tester.ad_9361[4]->tune("TX",50e6,false);
//            tester.ad_9361[4]->tune("TX",1000e6,false);
//            tester.ad_9361[4]->tune("TX",2400e6,false);*/

//        /*while(1){
//            //for(int i = 1; i < 8; i+=4){
//            int i = 1;

//            tester.ad_9361[1]->tune("RX",60e6,true);
//            tester.ad_9361[1]->tune("RX",450e6,true);
//            tester.ad_9361[1]->tune("RX",60e6,true);




//                tester.filter_bank[i]->do_rx_attenuation(0);
//                tester.filter_bank[i]->set_filter_path_simulator(60e6, "RX", false);
////                tester.filter_bank[i]->set_rx_path_throw_outside_flt(true);
////                tester.filter_bank[i]->set_rx_path_throw_outside_flt(false);
//                tester.filter_bank[i]->set_filter_path_simulator(130e6, "RX", false);
////                for(uint8_t j = 0; j<50;j+=3)
////                    tester.filter_bank[i]->do_rx_attenuation(j);
////                tester.filter_bank[i]->do_rx_attenuation(0);
//                tester.filter_bank[i]->set_filter_path_simulator(264e6, "RX", false);
//                tester.filter_bank[i]->set_filter_path_simulator(470e6, "RX", false);
//                tester.filter_bank[i]->set_filter_path_simulator(1000e6, "RX", false);
//                tester.filter_bank[i]->set_filter_path_simulator(2400e6, "RX", false);
//                tester.filter_bank[i]->set_filter_path_simulator(3000e6, "RX", false);
//            }*/
//        }


////        uint32_t    temp =  tester.iface->peek32(READBACK_BASE + 4*1);
////        tester.iface->poke32(U2_REG_SR_ADDR(1001),0x1FFF);
////        tester.iface->poke32(U2_REG_SR_ADDR(1000),3);
////        tester.iface->poke32(U2_REG_SR_ADDR(1000),2);
////        tester.iface->poke32(U2_REG_SR_ADDR(1000),1);

////        while (true){
////            tester.iface->poke32(U2_REG_SR_ADDR(1000),3);
////            tester.iface->poke32(U2_REG_SR_ADDR(1000),2);
////            tester.iface->poke32(U2_REG_SR_ADDR(1000),1);
////        }


////        tester.iface->poke32(U2_REG_SR_ADDR(101),1); //GO TO 12V
////        tester.iface->poke32(U2_REG_SR_ADDR(101),2); //GO TO 15V
////    while(true){
////        tester.iface->poke32(U2_REG_SR_ADDR(100),0);    //CH SW TO 0
////        tester.iface->poke32(U2_REG_SR_ADDR(100),1);
////        tester.iface->poke32(U2_REG_SR_ADDR(100),2);
////        tester.iface->poke32(U2_REG_SR_ADDR(100),3);
////        tester.iface->poke32(U2_REG_SR_ADDR(100),4);
////        tester.iface->poke32(U2_REG_SR_ADDR(100),5);
////        tester.iface->poke32(U2_REG_SR_ADDR(100),6);
////        tester.iface->poke32(U2_REG_SR_ADDR(100),7);
////    }


////        while(true){
////            tester.time64->set_time_next_pps(pax::time_spec_t(0.00));
////            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
////            pax::time_spec_t t =tester.time64->get_time_last_pps();
////            long long time = t.get_tick_count(DSP_CLOCK_RATE);
////            std::cout<< time <<std::endl;
////        }

////        tester.gps = pax::gps::make(tester.iface,2);
////        tester.gps->run();
////        tester.compass = pax::compass::make(tester.iface,3);
////        tester.compass->run();
////    while(true){
////        std::cout<<"HEADING   "<<tester.compass->get_heading() <<std::endl;
////        std::cout<<"LAT   "<<tester.gps->get_lat() <<std::endl;
////        std::cout<<"LON   "<<tester.gps->get_lon() <<std::endl;
////        std::cout<<"H   "<<tester.gps->get_GMT_hour() <<std::endl;
////        std::cout<<"M   "<<tester.gps->get_GMT_minute() <<std::endl;
////        std::cout<<"S   "<<tester.gps->get_GMT_second() <<std::endl;
////        std::cout<<"############"<<std::endl;

////        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
////    }

////        pax::flash_vec_t flash_vec;
////        for(uint32_t i=0;i<(1<<17);i++){
////            uint8_t oo=i;
////            flash_vec.push_back(255-oo);
////        }


////    tester.iface->erase_flash(0,flash_vec.size());
////    tester.iface->write_flash(0,flash_vec);
////    tester.iface->read_flash(0,flash_vec,flash_vec.size());


////    for(uint32_t i=0;i<(1<<17);i++){
////        uint8_t oo=i;
////        if(flash_vec[i] != (255-oo))
////            throw "FUCK";
////    }
////    throw "FUCK";


////    tester.iface->erase_otp_flash(pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
////    tester.iface->write_otp_flash(0,flash_vec,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
////    tester.iface->read_otp_flash(0,flash_vec,20,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
////    tester.iface->read_otp_flash(0,flash_vec,20,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
////    flash_vec = tester.iface->read_uid_flash();


////    find_all_pax_ip();
////    pax::pax_reload::sptr reloader = pax::pax_reload::make(tester.iface);
////    reloader->burn_fpga_image(0,"/home/parto/Desktop/pax/PAX2/pax2_k7/v2/PAX2K7/pax2_fw_lock.bit");


//    return 0;
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "recorder.hpp"
//#include <eeprom.hpp> // /////////////////////////////////////////////
#include <pax_reload.hpp>
#include "ad9361_ctrl.hpp"
#include <string>

int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;
//pax::usrp:: test1;
        mb_container_type tester;
        std::vector<boost::shared_ptr<pax::transport::sph::recv_packet_streamer> > streamers=pax_init(tester,8);//8/////////
//#define _AD9361 0
//#define _TEST_FREQ (50e6)
 //       tester.sync->GNS_calibration(950e6 , 50e6 , true);
        tester.filter_bank[1]->disable_all_regulators(true);
        while(1){

            tester.filter_bank[5]->set_filter_path(1000e6 , "RX" , false );
            tester.filter_bank[5]->disable_all_regulators(true);
            tester.filter_bank[5]->set_filter_path(1900e6 , "RX" , false );
            tester.filter_bank[5]->disable_all_regulators(true);

tester.filter_bank[5]->set_filter_path(2500e6 , "RX" , false );
tester.filter_bank[5]->disable_all_regulators(true);
tester.filter_bank[5]->set_filter_path(5000e6 , "RX" , false );
tester.filter_bank[5]->disable_all_regulators(true);
//tester.filter_bank[5]->set_filter_path(2000e6 , "RX" , false );
//tester.filter_bank[5]->disable_all_regulators(true);
//tester.filter_bank[5]->set_filter_path(2000e6 , "RX" , false );
//tester.filter_bank[5]->disable_all_regulators(true);


        }
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);

//{
    double ADSampleRate;
     for(uint32_t i=1;i<8/*tester.ad_9361.size()*/;i=i+2)// //////////////////////
     {
         tester.ad_9361[i]->set_active_chains(true,false,true,false);//true
 //         // internal lo (set by default)
         tester.ad_9361[i]->tune("RX",500e6/*+2000*/);
         tester.ad_9361[i]->set_agc("RX1",false);
         tester.ad_9361[i]->set_gain("RX1",50);
 //         tester.ad_9361[i]->set_gain("RX2",10);

         tester.ad_9361[i]->tune("TX",500e6);
         tester.ad_9361[i]->set_gain("TX1",50);

         ADSampleRate=tester.ad_9361[i]->set_clock_rate(60e6);
         tester.ad_9361[i]->output_digital_test_prbs(false);
//tester.ad_9361[i]->output_digital_test_tone(false);
         tester.ad_9361[i]->data_port_loopback(true);
     }
     // //////////////////////////////////////////////////////////////
//     tester.ad_9361[1]->set_active_chains(false,false,true,false);//true
// //         // internal lo (set by default)
//     tester.ad_9361[1]->tune("RX",2000e6/*+2000*/);
//     tester.ad_9361[1]->set_agc("RX1",false);
//     tester.ad_9361[1]->set_gain("RX1",50);
// //         tester.ad_9361[i]->set_gain("RX2",10);

// //     tester.ad_9361[i]->tune("TX",2000e6);
// //     tester.ad_9361[i]->set_gain("TX1",50);

//     ADSampleRate=tester.ad_9361[1]->set_clock_rate(50e6);
//     tester.ad_9361[1]->output_digital_test_prbs(false);
// //tester.ad_9361[i]->output_digital_test_tone(false);
//     tester.ad_9361[1]->data_port_loopback(false);
     // //////////////////////////////////////////
     for(uint32_t i=0;i<8/*tester.ad_9361.size()*/;i=i+2)// //////////////////////
     {
         tester.ad_9361[i]->set_active_chains(true,false,true,false);//true
    //      // internal lo (set by default)
         tester.ad_9361[i]->tune("RX",500e6/*+2000*/);
         tester.ad_9361[i]->set_agc("RX1",false);
         tester.ad_9361[i]->set_gain("RX1",50);
//     //     tester.ad_9361[i]->set_gain("RX2",10);

         tester.ad_9361[i]->tune("TX",500e6);
         tester.ad_9361[i]->set_gain("TX1",50);

         ADSampleRate=tester.ad_9361[i]->set_clock_rate(60e6);
         tester.ad_9361[i]->output_digital_test_prbs(false);
//tester.ad_9361[i]->output_digital_test_tone(false);
         tester.ad_9361[i]->data_port_loopback(true);
     }
//       tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x1);

//  tester.ad_9361[7]->data_port_loopback(true);
//    tester.ad_9361[3]->data_port_loopback(true);
//      tester.ad_9361[1]->data_port_loopback(true);
//        tester.ad_9361[5]->data_port_loopback(true);

         // enable clock from ad9361


//         tester.ad_9361[0]->data_port_loopback(true);
//         tester.ad_9361[1]->data_port_loopback(true);
//         tester.ad_9361[2]->data_port_loopback(true);
//         tester.ad_9361[3]->data_port_loopback(true);
//         tester.ad_9361[4]->data_port_loopback(true);
//         tester.ad_9361[5]->data_port_loopback(true);
//         tester.ad_9361[6]->data_port_loopback(true);
//         tester.ad_9361[7]->data_port_loopback(true);

//tester.ad_9361[0]->output_digital_test_prbs(true);
//tester.ad_9361[1]->output_digital_test_prbs(true);
//tester.ad_9361[2]->output_digital_test_prbs(true);
//tester.ad_9361[3]->output_digital_test_prbs(true);
//tester.ad_9361[4]->output_digital_test_prbs(true);
//tester.ad_9361[5]->output_digital_test_prbs(true);
//tester.ad_9361[6]->output_digital_test_prbs(true);
//tester.ad_9361[7]->output_digital_test_prbs(true);
//         tester.ad_9361[0]->output_digital_test_tone(true);
//         tester.ad_9361[1]->output_digital_test_tone(true);

//         tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN_A) , 0xFF );

        std::cout<<"real sample_rate:  "<<ADSampleRate<<std::endl;

//        for (size_t i=0; i<streamers.size(); i++){
//            tester.rx_dsps[i]->set_tick_rate(ADSampleRate);
//            tester.rx_dsps[i]->set_link_rate(ADSampleRate);
//            tester.rx_dsps[i]->set_host_rate(2e6);
//            tester.rx_dsps[i]->set_freq(-5e6);
//        }
        tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x1);

//        tester.iface->poke32(U2_REG_SR_ADDR(990), 0x01);
//        tester.iface->poke32(U2_REG_SR_ADDR(991), 0x01);


//        // setup dds
//         tester.iface->poke32(U2_REG_SR_ADDR(980),1);
//         tester.iface->poke32(U2_REG_SR_ADDR(981),(0x1fff<<16)|0x1fff); // scale_0
//         tester.iface->poke32(U2_REG_SR_ADDR(982),(0<<16)|32768>>1); // phase_0
//         tester.iface->poke32(U2_REG_SR_ADDR(983),((32768>>5)<<16)|32768>>5); // incr_0
//         //tester.iface->poke32(U2_REG_SR_ADDR(983),((0>>6)<<16)|0>>6); // incr_0
//         tester.iface->poke32(U2_REG_SR_ADDR(984),((0<<16)>>0)|(0>>0)); // scale_1
//         tester.iface->poke32(U2_REG_SR_ADDR(985),(0<<16)|0>>1); // phase_1
//         tester.iface->poke32(U2_REG_SR_ADDR(986),(0)|0); // incr_1
//         tester.iface->poke32(U2_REG_SR_ADDR(980),0);


     // setup dds
     tester.iface->poke32(U2_REG_SR_ADDR(980),1);
     tester.iface->poke32(U2_REG_SR_ADDR(981),(0x3fff<<16)|0x3fff); // scale_0
     tester.iface->poke32(U2_REG_SR_ADDR(982),(0<<16)|32768>>1); // phase_0
     tester.iface->poke32(U2_REG_SR_ADDR(983),((32768>>4)<<16)|32768>>4); // incr_0
     //tester.iface->poke32(U2_REG_SR_ADDR(983),((0>>6)<<16)|0>>6); // incr_0
     tester.iface->poke32(U2_REG_SR_ADDR(984),(0<<16)|0); // scale_1
     tester.iface->poke32(U2_REG_SR_ADDR(985),(0<<16)|0); // phase_1
     tester.iface->poke32(U2_REG_SR_ADDR(986),(0<<16)|0); // incr_1
     tester.iface->poke32(U2_REG_SR_ADDR(980),0);

//     tester.iface->poke32(U2_REG_SR_ADDR(255),0x0);

//         //alan bayad regport 981 ro be module dds vasl konaM?
         tester.iface->poke32(U2_REG_SR_ADDR(255),0xffff);


//        pax::rx_metadata_t md;
//        int num_requested_samples=10000;

//        pax::stream_cmd_t stream_cmd((num_requested_samples == 0)?
//                                         pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
//                                         pax::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
//                                         );

//        stream_cmd.num_samps = 1;
//        stream_cmd.stream_now = true;
//        stream_cmd.stream_mode=pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
//        stream_cmd.time_spec = pax::time_spec_t();

// //        tester.rx_dsps[0]->issue_stream_command(stream_cmd);
// //        tester.ad_9361[0]->set_clk_mux_parent(1);
//        for (size_t i=0; i<2/*4*/; i++)
//            tester.rx_dsps[i]->issue_stream_command(stream_cmd);

//        std::vector<boost::thread*> runners(streamers.size());
//        std::vector<recorder::sptr> recorders(streamers.size());

//        for (size_t i=0; i<2; i++){
//            recorders[i]=recorder::make(streamers[i],i,200000);
//            runners[i]=new boost::thread(boost::bind(&recorder::run,recorders[i]));

//        }

//        runners[0]->join();
//        streamers.data();

//        tester.ad_9361[0]->output_digital_test_prbs(true);
//        tester.ad_9361[1]->output_digital_test_prbs(true);



















//tester.iface->poke32(U2_REG_SR_ADDR(1000),2);
//tester.iface->poke32(U2_REG_SR_ADDR(1000),100);
//tester.iface->poke32(U2_REG_SR_ADDR(1000),500);
//tester.iface->poke32(U2_REG_SR_ADDR(1000),1000);
//tester.iface->poke32(U2_REG_SR_ADDR(1000),2000);
//tester.iface->poke32(U2_REG_SR_ADDR(1000),3);
//output_digital_test_prbs(1);


    // disable clock from ad9361
//    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xf);

//    double ADSampleRate;

//        for (uint32_t i=0; i<(tester.N_AD9361 + tester.N_AD9364); i++){
//            tester.ad_9361[i]->set_active_chains(true,false,true,false);


//           tester.ad_9361[i]->set_gain("TX2",85);///////////
//          ADSampleRate=tester.ad_9361[i]->set_clock_rate(50e6);////////
//            ADSampleRate=tester.ad_9361[i]->set_clock_rate(50e6);
//             tester.ad_9361[i]->set_active_chains(true,false,true,false);
//             tester.ad_9361[i]->tune("TX",_TEST_FREQ);
//             tester.ad_9361[i]->set_gain("TX1",89);
//            tester.ad_9361[i]->swap_iq("TX",true);////////
//           tester.ad_9361[i]->output_digital_test_tone(true);//////////
//            tester.ad_9361[i]->output_digital_test_prbs(true);
//        }
//tester.ad_9361[0]->set_clock_rate(50e6);
//tester.ad_9361[0]->set_active_chains(true,false,true,false);
//tester.ad_9361[0]->tune("TX",_TEST_FREQ);
//tester.ad_9361[0]->set_gain("TX1",89);

//tester.ad_9361[0]->tune("RX",20e6);
//tester.ad_9361[0]->set_agc("RX1",false); // automatic gain control _ peak to be const
//tester.ad_9361[0]->set_gain("RX1",30);

//tester.ad_9361[0]->output_digital_test_prbs(true);

//tester.ad_9361[1]->set_clock_rate(50e6);
//tester.ad_9361[1]->set_active_chains(true,false,true,false);
//tester.ad_9361[1]->tune("TX",_TEST_FREQ);
//tester.ad_9361[1]->set_gain("TX1",89);

//tester.ad_9361[1]->tune("RX",20e6);
//tester.ad_9361[1]->set_agc("RX1",false); // automatic gain control _ peak to be const
//tester.ad_9361[1]->set_gain("RX1",30);



//    }
     //       tester.iface->poke32(U2_REG_SR_ADDR(104), 1<<0);// to_wb
    //        tester.fifo_ctrl->poke32(U2_REG_SR_ADDR(104), 1<<0);// sfc_without_wb

    //        while(1) std::cout << tester.fifo_ctrl->peek32(U2_REG_SR_ADDR(24)) << std::endl;

    //        std::cout <<1;

            //tester.filter_bank[4]->set_filter_path_simulator(1000e6, "RX", false);
            //std::cout << "ok!";


    //        while(1){
    //            for(int i = 0; i < 3000; i++)
    //                for(int j = 2; j < 4; j++)
    //                    tester.ad_9361[j]->tune("RX",(50+i)*1e6);
    //        }
            /*pax::flash_vec_t x = tester.iface->read_uid_flash();
            std::cout << "%%%%%%%%" << std::endl;

            for(int i = 0; i < x.size(); i++)
                std::cout << x[i] << std::endl;*/

            //tester.iface->erase_flash(0x0,10);
            //tester.iface->write_flash(0x5,582);
            //pax::flash_vec_t uid = tester.iface->read_uid_flash();//PH


            //pax::pax_reload::sptr reloader = pax::pax_reload::make(tester.iface);
            //reloader->burn_fpga_image(0,"/home/parto/Desktop/pax/PAX8/Pax8V7/PAX8_V7/PAX8_V7_fw_UNLOCK_.bit");

            //tester.iface->_bpi_write_reg(5,666);
            //std::cout << "****" << tester.iface->_bpi_read_reg(5);

            /*{

                tester.iface->cfi_read_all_device_ident();
                tester.iface->cfi_queries();
                tester.iface->cfi_unlock_and_erase_block(0);


                tester.iface->_bpi_write_reg(5,666);
                std::cout << "************" << tester.iface->_bpi_read_reg(0xE01004E) << std::endl;


                std::cout << "begin: " << std::endl;
                clock_t begin = clock();
                for(int i=0; i<8; i++) {
                    tester.iface->cfi_data_write_32bits(0x00000000 + 4*i, 0x000c0000 + i);
                }
                clock_t end = clock();
                double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                std::cout << "write time: " << elapsed_secs<<std::endl;

                begin = clock();
                for(int i=0; i<8; i++) {
                    uint32_t t1 = tester.iface->cfi_read_16_bits(0x00000000 +4*i);
                    uint32_t t2 = tester.iface->cfi_read_16_bits(0x00000000 +4*i+2);
                    uint32_t res = t2 + (t1 << 16);
                    if(res != (0x000c0000 + i))
                        std::cout << t1 << "    ___   " << t2 << std::endl;
                }
                end = clock();
                std::cout << "end: " << std::endl;
                elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                std::cout << "read time: " << elapsed_secs<<std::endl;
                uint32_t time = tester.iface->cfi_read_32_bits(32*1024*4);
                std::cout<<time<<std::endl;

            }*/













    //       tester.iface->poke32(U2_REG_SR_ADDR(1022),0);
    //        tester.iface->poke32(U2_REG_SR_ADDR(1023),65535);

    //        while(true){
    //        tester.iface->poke32(U2_REG_SR_ADDR(101),0x0);
    //        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    //        tester.iface->poke32(U2_REG_SR_ADDR(101),0x1);
    //        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    //}


    //        while(1){
    //            std::cout << tester.iface->peek32(U2_REG_SR_ADDR(101)) << std::endl;
    //            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    //        }
            //tester.sync->PAX8V7_calibration();
            //      tester.iface->poke32(U2_REG_SR_ADDR(100),8); //GO TO 12V


            //        typedef enum {low_pass_40MHz=0,low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz,low_pass_2200MHz,low_pass_6000MHz} FILTER_PATH_virtex;



//            int which_9364 = 4;
//            long int freq = 5800e6;
//            int gain = 50;


//            for (uint32_t i=0; i<(tester.N_AD9361 + tester.N_AD9364); i++){
//                if(i != which_9364){
//                tester.ad_9361[i]->set_active_chains(false,false,false,false);
//                tester.ad_9361[i]->tune("TX",freq,true);
//                tester.ad_9361[i]->set_gain("TX1",0);
//                tester.ad_9361[i]->output_digital_test_tone(false);
//                }
//            }

//            tester.ad_9361[which_9364]->set_active_chains(true,false,false,false);
//            tester.ad_9361[which_9364]->tune("TX",freq,true);
//            tester.ad_9361[which_9364]->set_gain("TX1",gain);
//            tester.ad_9361[which_9364]->output_digital_test_tone(true);


//            // enable clock from ad93611
//            tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xff);


//            std::cout<<"1";

//            while(true){
//                /*tester.filter_bank[0]->set_filter_path_simulator(900e6,"TX",false);
//                tester.ad_9361[0]->tune("TX",50e6,false);
//                tester.ad_9361[0]->tune("TX",500e6,false);
//                tester.ad_9361[0]->tune("TX",1000e6,false);
//                tester.filter_bank[0]->set_filter_path_simulator(2200e6,"TX",false);
//                tester.ad_9361[0]->tune("TX",50e6,false);
//                tester.ad_9361[0]->tune("TX",1000e6,false);
//                tester.ad_9361[0]->tune("TX",2400e6,false);

//                tester.filter_bank[2]->set_filter_path_simulator(100e6,"TX",false);
//                tester.ad_9361[2]->tune("TX",50e6,false);
//                tester.ad_9361[2]->tune("TX",90e6,false);
//                tester.ad_9361[2]->tune("TX",130e6,false);
//                tester.filter_bank[2]->set_filter_path_simulator(200e6,"TX",false);
//                tester.ad_9361[2]->tune("TX",50e6,false);
//                tester.ad_9361[2]->tune("TX",100e6,false);
//                tester.ad_9361[2]->tune("TX",264e6,false);
//                tester.filter_bank[2]->set_filter_path_simulator(300e6,"TX",false);
//                tester.ad_9361[2]->tune("TX",50e6,false);
//                tester.ad_9361[2]->tune("TX",200e6,false);
//                tester.ad_9361[2]->tune("TX",470e6,false);
//                tester.filter_bank[2]->set_filter_path_simulator(500e6,"TX",false);
//                tester.ad_9361[2]->tune("TX",50e6,false);
//                tester.ad_9361[2]->tune("TX",500e6,false);
//                tester.ad_9361[2]->tune("TX",1000e6,false);



//                tester.filter_bank[4]->set_filter_path_simulator(900e6,"TX",false);
//                tester.ad_9361[4]->tune("TX",50e6,false);
//                tester.ad_9361[4]->tune("TX",500e6,false);
//                tester.ad_9361[4]->tune("TX",1000e6,false);
//                tester.filter_bank[4]->set_filter_path_simulator(2200e6,"TX",false);
//                tester.ad_9361[4]->tune("TX",50e6,false);
//                tester.ad_9361[4]->tune("TX",1000e6,false);
//                tester.ad_9361[4]->tune("TX",2400e6,false);*/

//            /*while(1){
//                //for(int i = 1; i < 8; i+=4){
//                int i = 1;

//                tester.ad_9361[1]->tune("RX",60e6,true);
//                tester.ad_9361[1]->tune("RX",450e6,true);
//                tester.ad_9361[1]->tune("RX",60e6,true);




//                    tester.filter_bank[i]->do_rx_attenuation(0);
//                    tester.filter_bank[i]->set_filter_path_simulator(60e6, "RX", false);
//    //                tester.filter_bank[i]->set_rx_path_throw_outside_flt(true);
//    //                tester.filter_bank[i]->set_rx_path_throw_outside_flt(false);
//                    tester.filter_bank[i]->set_filter_path_simulator(130e6, "RX", false);
//    //                for(uint8_t j = 0; j<50;j+=3)
//    //                    tester.filter_bank[i]->do_rx_attenuation(j);
//    //                tester.filter_bank[i]->do_rx_attenuation(0);
//                    tester.filter_bank[i]->set_filter_path_simulator(264e6, "RX", false);
//                    tester.filter_bank[i]->set_filter_path_simulator(470e6, "RX", false);
//                    tester.filter_bank[i]->set_filter_path_simulator(1000e6, "RX", false);
//                    tester.filter_bank[i]->set_filter_path_simulator(2400e6, "RX", false);
//                    tester.filter_bank[i]->set_filter_path_simulator(3000e6, "RX", false);
//                }*/
//            }


//    //        uint32_t    temp =  tester.iface->peek32(READBACK_BASE + 4*1);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(1001),0x1FFF);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(1000),3);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(1000),2);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(1000),1);

//    //        while (true){
//    //            tester.iface->poke32(U2_REG_SR_ADDR(1000),3);
//    //            tester.iface->poke32(U2_REG_SR_ADDR(1000),2);
//    //            tester.iface->poke32(U2_REG_SR_ADDR(1000),1);
//    //        }


//    //        tester.iface->poke32(U2_REG_SR_ADDR(101),1); //GO TO 12V
//    //        tester.iface->poke32(U2_REG_SR_ADDR(101),2); //GO TO 15V
//    //    while(true){
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),0);    //CH SW TO 0
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),1);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),2);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),3);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),4);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),5);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),6);
//    //        tester.iface->poke32(U2_REG_SR_ADDR(100),7);
//    //    }


//    //        while(true){
//    //            tester.time64->set_time_next_pps(pax::time_spec_t(0.00));
//    //            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//    //            pax::time_spec_t t =tester.time64->get_time_last_pps();
//    //            long long time = t.get_tick_count(DSP_CLOCK_RATE);
//    //            std::cout<< time <<std::endl;
//    //        }

//    //        tester.gps = pax::gps::make(tester.iface,2);
//    //        tester.gps->run();
//    //        tester.compass = pax::compass::make(tester.iface,3);
//    //        tester.compass->run();
//    //    while(true){
//    //        std::cout<<"HEADING   "<<tester.compass->get_heading() <<std::endl;
//    //        std::cout<<"LAT   "<<tester.gps->get_lat() <<std::endl;
//    //        std::cout<<"LON   "<<tester.gps->get_lon() <<std::endl;
//    //        std::cout<<"H   "<<tester.gps->get_GMT_hour() <<std::endl;
//    //        std::cout<<"M   "<<tester.gps->get_GMT_minute() <<std::endl;
//    //        std::cout<<"S   "<<tester.gps->get_GMT_second() <<std::endl;
//    //        std::cout<<"############"<<std::endl;

//    //        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//    //    }

//    //        pax::flash_vec_t flash_vec;
//    //        for(uint32_t i=0;i<(1<<17);i++){
//    //            uint8_t oo=i;
//    //            flash_vec.push_back(255-oo);
//    //        }


//    //    tester.iface->erase_flash(0,flash_vec.size());
//    //    tester.iface->write_flash(0,flash_vec);
//    //    tester.iface->read_flash(0,flash_vec,flash_vec.size());


//    //    for(uint32_t i=0;i<(1<<17);i++){
//    //        uint8_t oo=i;
//    //        if(flash_vec[i] != (255-oo))
//    //            throw "FUCK";
//    //    }
//    //    throw "FUCK";


//    //    tester.iface->erase_otp_flash(pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
//    //    tester.iface->write_otp_flash(0,flash_vec,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
//    //    tester.iface->read_otp_flash(0,flash_vec,20,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
//    //    tester.iface->read_otp_flash(0,flash_vec,20,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
//    //    flash_vec = tester.iface->read_uid_flash();


//    //    find_all_pax_ip();
//    //    pax::pax_reload::sptr reloader = pax::pax_reload::make(tester.iface);
//    //    reloader->burn_fpga_image(0,"/home/parto/Desktop/pax/PAX2/pax2_k7/v2/PAX2K7/pax2_fw_lock.bit");


        return 0;
    }






