//
// Copyright 2010-2016 Parto
//

#include <pax_sync.h>

#include <device_addr.hpp>

#include <fw_common.h>
#include <exception.hpp>
#include <dict.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp> //used for htonl and ntohl
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/functional/hash.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iostream>
#include <platform.hpp>
#include <tasks.hpp>
#include <pax_regs.hpp>
#include <boost/range/algorithm.hpp>
#include <exception.hpp>
#include <boost/lexical_cast.hpp>
#include <bitset>
#include <stdint.h>
#include <boost/math/constants/constants.hpp>
#include <filter_bank.hpp>
#include <filter_bank_imple.hpp>
class PAX_API pax_sync_impl : public pax::pax_sync{
public:
    /***********************************************************************
 * Structors
 **********************************************************************/
    pax_sync_impl(std::vector<pax::usrp::ad9361_ctrl::sptr>& vAD9361, pax::wb_iface::sptr wb_iface):
        vAD9361(vAD9361),
        _wb_iface(wb_iface)
    {
        // TODO: check revision
    }

    ~pax_sync_impl(void){}
    /***********************************************************************
     * Calibration
     **********************************************************************/
    void do_mcs(){
        if(vAD9361.empty()==true)
            return;
        for (size_t i=0; i<vAD9361.size(); i++)
        {
            vAD9361[i]->ensm_force_alert();
            boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        }
        for (size_t step=0; step<6; step++){
            if (step!=2 && step!=4)
                for (size_t i=0; i<vAD9361.size(); i++)
                {
                    vAD9361[i]->do_mcs(step);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
                }
            else
            {
                vAD9361[0]->do_mcs(step);
                boost::this_thread::sleep(boost::posix_time::milliseconds(5));
            }
        }
        for (size_t i=0; i<vAD9361.size(); i++)
        {
            std::cout << i << std::endl;
            vAD9361[i]->ensm_force_fdd();
        }

        return;
    }
    void PAX8K7_set_accuracy_value_of_phase_cal(double val){
        pax8_accurecy_of_phase_calibration_degree = val;
    }
    double PAX8K7_get_accuracy_value_of_phase_cal(){
        return pax8_accurecy_of_phase_calibration_degree;
    }

    rev_type get_rev(void){
        std::string hw = "";//mb_eeprom["hardware"];
        if (hw.empty()) return PAX_UNKNOWN;
        switch (boost::lexical_cast<boost::uint16_t>(hw)){
        case 0x1800: return PAX8_REV0;
            //        case 0x0301: return USRP2_REV3;
            //        case 0x0400: return USRP2_REV4;
            //        case 0x0A00: return USRP_N200;
            //        case 0x0A01: return USRP_N210;
            //        case 0x0A10: return USRP_N200_R4;
            //        case 0x0A11: return USRP_N210_R4;
        }
        return PAX_UNKNOWN; //unknown type
    }
    const std::string get_cname(void){
        switch(this->get_rev()){
        case PAX8_REV0: return "PAX8";
        case PAX_UNKNOWN: return "UNKNOWN";
        }
        PAX_THROW_INVALID_CODE_PATH();
    }

    std::vector<double> PAX8K7_calibration(double _TEST_FREQ = 1575e6 ,double SAMPLE_RATE = 32e6,bool test_mode = true){
        std::vector<double> out_value;

        if(vAD9361.empty()==true)
            return out_value;
        init_for_calibration_PAX8K7(_TEST_FREQ,SAMPLE_RATE,test_mode);
         PAX8K7_rx_cal_mode(true);
        tx_sw_ch_for_cal_pax8(0);
        int failed_iterator=0;double I_delay,Q_delay;double teta=0;
        bool extra_calibrate=false;
        for(uint16_t i=0;i<8;){
           if(!extra_calibrate){
               normalize_and_commit_I_Q_PAX8(0,i);
               boost::this_thread::sleep(boost::posix_time::microseconds(200));
           }
           read_I_Q_CIC_GNS(I_delay,Q_delay);
           teta+=atan2(Q_delay,I_delay);
           normalize_and_commit_I_Q_PAX8(teta,i);
           if(check_phase_calibration_met(pax8_accurecy_of_phase_calibration_degree,I_delay,Q_delay)){
//               if(test_mode)
               std::cout << "teta" << i << ": " << teta*_degree_180/pi << std::endl;
                std::cerr<<"PHASE reported  "<<i<<"     "<< std::atan2(Q_delay,I_delay)*_degree_180/pi<<std::endl;
                    i++;failed_iterator=0;teta=0;extra_calibrate=false;
               tx_sw_ch_for_cal_pax8(i);
               out_value.push_back(std::atan2(Q_delay,I_delay)*_degree_180/pi);
           } else {
               failed_iterator++;extra_calibrate=true;
               if(failed_iterator%50==0 && failed_iterator!=0){
                   teta=0;extra_calibrate=false;
               }
               if(failed_iterator == 100){
                   std::cerr<<"channel "<<i<<" can not calibrate ! "<< "PHASE REPORT: "<< std::atan2(Q_delay,I_delay)*_degree_180/pi <<std::endl;
                   i++;
                   tx_sw_ch_for_cal_pax8(i);
                   out_value.push_back(std::atan2(Q_delay,I_delay)*_degree_180/pi);
                   failed_iterator=0;extra_calibrate=false;
                   teta=0;
                   continue;
               }
           }
        }

        vAD9361[2]->set_gain("TX1",0);
        vAD9361[2]->set_gain("TX2",0);
       PAX8K7_rx_cal_mode(false);
       _wb_iface->poke32(U2_REG_SR_ADDR(255),0x00);
       //vAD9361[2]->set_active_chains(false,false,true,true);
       return out_value;
    }


    void set_filter_bank(std::vector<pax::filter_bank::sptr> flt){ // PH
        for(uint8_t k = 0; k < flt.size(); k++)
            filter.push_back(flt[k]);
    }
    std::vector<double> PAX8V7_calibration(double _TEST_FREQ = 1575e6 ,double SAMPLE_RATE = 32e6,bool test_mode = true){
        std::vector<double> out_value;
        std::vector<double> commited_teta;

        if(vAD9361.empty()==true)
            return out_value;


        // for(uint8_t j = 0; j < filter.size(); j++)
        //     filter[j]->set_filter_path_virtex(_TEST_FREQ,false);// PH

        for(uint8_t j = 0; j < filter.size(); j++)
            filter[j]->set_filter_path(_TEST_FREQ,"RX" ,false);// PH


        init_for_calibration_PAX8V7(_TEST_FREQ,SAMPLE_RATE,test_mode);

        PAX8V7_rx_cal_mode(true);
        tx_sw_ch_for_cal_pax8(0);
        for(uint8_t k = 0; k < 8; k++){
            normalize_and_commit_I_Q_PAX8(0,k);
            boost::this_thread::sleep(boost::posix_time::microseconds(200));
        }
        int failed_iterator=0;double I_delay,Q_delay;double teta=0;
        bool extra_calibrate=false;
        for(uint16_t i=0;i<8;){
           if(!extra_calibrate){
               normalize_and_commit_I_Q_PAX8(0,i);
               boost::this_thread::sleep(boost::posix_time::microseconds(200));
           }
           read_I_Q_CIC_PAX8V7(I_delay,Q_delay);


           teta=std::atan2(Q_delay,I_delay);

           normalize_and_commit_I_Q_PAX8(teta,i);

           if(check_phase_calibration_PAX8V7_met(pax8_accurecy_of_phase_calibration_degree,I_delay,Q_delay)){
               std::cerr<<"PHASE reported  "<<i<<"     "<< std::atan2(Q_delay,I_delay)*_degree_180/pi<<std::endl;
               std::cerr << "commited "  << i << ":  " << teta*_degree_180/pi << std::endl;
               i++;
               commited_teta.push_back(teta);
               failed_iterator=0;teta=0;extra_calibrate=false;
               tx_sw_ch_for_cal_pax8(i);
               out_value.push_back(std::atan2(Q_delay,I_delay)*_degree_180/pi);
           } else {
               failed_iterator++;extra_calibrate=true;
               if(failed_iterator%10==0 && failed_iterator!=0){ //50 PH
                   teta=0;extra_calibrate=false;
               }
               if(failed_iterator == 20){//100 PH
                   std::cerr<<"channel "<<i<<" can not calibrate ! "<< "PHASE REPORT: "<< std::atan2(Q_delay,I_delay)*_degree_180/pi <<std::endl;
                   std::cerr << "commited "  << i << ":  " << teta*_degree_180/pi << std::endl;

                   i++;
                   tx_sw_ch_for_cal_pax8(i);
                   commited_teta.push_back(teta);
                   out_value.push_back(std::atan2(Q_delay,I_delay)*_degree_180/pi);
                   failed_iterator=0;extra_calibrate=false;
                   teta=0;
                   continue;
               }
           }
        }


        normalize_and_commit_I_Q_PAX8(-pi + commited_teta[2],3);
//        normalize_and_commit_I_Q_PAX8(((-pi + commited_teta[0])+commited_teta[1])/2,1);
//        normalize_and_commit_I_Q_PAX8(-pi+((-pi + commited_teta[4])+commited_teta[5])/2,5);
//        normalize_and_commit_I_Q_PAX8(((-pi + commited_teta[6])+commited_teta[7])/2,7);
//        normalize_and_commit_I_Q_PAX8(-pi+((-pi + commited_teta[0])+commited_teta[1])/2,0);
//        normalize_and_commit_I_Q_PAX8(((-pi + commited_teta[4])+commited_teta[5])/2,4);
//        normalize_and_commit_I_Q_PAX8(-pi+((-pi + commited_teta[6])+commited_teta[7])/2,6);
        std::cerr<<"channel 3 phase, corrected to  "<<  out_value[2] <<std::endl;



        PAX8V7_rx_cal_mode(false);
        _wb_iface->poke32(U2_REG_SR_ADDR(255),0x00);
        //vAD9361[1]->set_active_chains(false,false,true,true);
        vAD9361[1]->set_gain("TX1", 0);
        vAD9361[1]->set_gain("TX2", 0);



        return out_value;

    }

    double read_phase_loc_ant(int channel){
        tx_sw_ch_for_cal_pax8(channel);
        double I_delay,Q_delay, teta;
        read_I_Q_CIC_GNS(I_delay,Q_delay);
        teta=std::atan2(Q_delay,I_delay);
        return teta * 180.0 / pi;
    }

    bool GNS_calibration(double _TEST_FREQ = 1575e6 ,double SAMPLE_RATE = 32e6,bool test_mode = false){
        if(vAD9361.empty()==true)
            return false;


         for(int i=0;i<8;i++)  // armin Add this
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),(1<<30));


         bool return_state = true;
         init_for_calibration_GNS(_TEST_FREQ,SAMPLE_RATE,true);
         int failed_iterator=0;
         double I_delay,Q_delay;
         double teta=0;
         bool extra_calibrate=false;
         for(uint8_t i=0;i<vAD9361.size();){
            if(!extra_calibrate){
                _wb_iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<30)));
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }


            sw_for_calibration(i);
            read_I_Q_CIC_GNS(I_delay,Q_delay);
            teta+=atan2(Q_delay,I_delay);
            normalize_and_commit_I_Q_GNS(teta,i);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            if(check_phase_calibration_met(gns_accurecy_of_phase_calibration_degree,I_delay,Q_delay)){
                if(true)
                 std::cout<<"PHASE reported  "<<(int)i<<"     "<< std::atan2(Q_delay,I_delay)*_degree_180/pi<<std::endl;
                i++;failed_iterator=0;teta=0;extra_calibrate=false;
            } else {
                failed_iterator++;extra_calibrate=true;
                if(failed_iterator%50==0 && failed_iterator!=0){
                    teta=0;extra_calibrate=false;

                }
                if(failed_iterator == 100){
                    std::cerr<<"channel "<<(int)(i)<<" can not calibrate ! "<< "PHASE REPORTE: "<< std::atan2(Q_delay,I_delay)*_degree_180/pi <<std::endl;
                    i++;
                    failed_iterator=0;extra_calibrate=false;
                    teta=0;
                    return_state = false;
                    continue;
                }
            }
         }
         if(test_mode){
             _wb_iface->poke32(U2_REG_SR_ADDR(255),0x00);

         }
         GNS_pass_channel();

         return return_state;

    }
    void GNS_pass_channel(){
        int write_reg =0;
        // write_reg |= (((1<<3)<<6) |(1<<3));
        // write_reg |= (((1<<1)<<6) |(1<<1));
        // _wb_iface->poke32(U2_REG_GPIO_ADDR(1),write_reg);
                write_reg |=  (1<<0) ;
                write_reg |=  (1<<1) ;

                _wb_iface->poke32(U2_REG_GPIO_ADDR(1),write_reg);
    }
private:

    /***********************************************************************
     * Peek and Poke
     **********************************************************************/
    void read_I_Q_CIC_GNS(double& I_delay,double& Q_delay){
        boost::int64_t zigma_i=0,zigma_q=0;
        for(uint32_t i=0;i<gns_cal_extra_median;i++){
            boost::this_thread::sleep(boost::posix_time::microseconds(200));
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_SNAPSHOT_CALIBRATION),1);
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_SNAPSHOT_CALIBRATION),0);
            zigma_i +=(static_cast<boost::int32_t>(_wb_iface->peek32(U2_PHASE_DELAY_I_VALUE)));
            zigma_q +=(static_cast<boost::int32_t>(_wb_iface->peek32(U2_PHASE_DELAY_Q_VALUE)));
        }
        I_delay = (static_cast<double>(zigma_i)/gns_cal_extra_median);
        Q_delay = (static_cast<double>(zigma_q)/gns_cal_extra_median);
    }
    void read_I_Q_CIC_PAX8V7(double& I_delay,double& Q_delay){
        boost::int64_t zigma_i=0,zigma_q=0;
        for(uint32_t i=0;i<gns_cal_extra_median;i++){
            boost::this_thread::sleep(boost::posix_time::microseconds(200));
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_SNAPSHOT_CALIBRATION),1);
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_SNAPSHOT_CALIBRATION),0);
            zigma_i +=(static_cast<boost::int32_t>(_wb_iface->peek32(U2_PHASE_DELAY_I_VALUE)));
            zigma_q +=(static_cast<boost::int32_t>(_wb_iface->peek32(U2_PHASE_DELAY_Q_VALUE)));
        }
        I_delay = -(static_cast<double>(zigma_i)/gns_cal_extra_median);
        Q_delay = (static_cast<double>(zigma_q)/gns_cal_extra_median);
    }

    void normalize_and_commit_I_Q_GNS(double teta,uint8_t channel){
        double I_delay,Q_delay;
        I_delay = std::cos(teta);
        Q_delay = std::sin(teta);
        I_delay *= -static_cast<double>(1<<14);
        Q_delay *= static_cast<double>(1<<14);
        uint32_t to_reg = ( (((int16_t) I_delay)<<16) | ((((int16_t)Q_delay)<<0)&0x0000ffff));
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(channel)),to_reg);
    }

    void normalize_and_commit_I_Q_PAX8(double teta,uint8_t channel){
        double I_delay,Q_delay;
        I_delay = std::cos(teta);
        Q_delay = std::sin(teta);
//        I_delay *= INT16_MIN ;
//        Q_delay *= INT16_MAX;
        uint32_t to_reg = ( (((int16_t) Q_delay)<<16) | ((((int16_t)I_delay)<<0)&0x0000ffff));
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(channel)),to_reg);
    }

    void initial_dds_for_calibration(){
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xFF);
        // setup dds
          _wb_iface->poke32(U2_REG_SR_ADDR(980),1);
          _wb_iface->poke32(U2_REG_SR_ADDR(981),(0x1fff<<16)|0x1fff); // scale_0
          _wb_iface->poke32(U2_REG_SR_ADDR(982),((0x0) <<16)|(0xffff>>2)); // phase_0
          _wb_iface->poke32(U2_REG_SR_ADDR(983),((32768>>7)<<16)|32768>>7); // incr_0
//             //_wb_iface->poke32(U2_REG_SR_ADDR(983),((0>>6)<<16)|0>>6); // incr_0
          _wb_iface->poke32(U2_REG_SR_ADDR(984),(0)); // scale_1
          _wb_iface->poke32(U2_REG_SR_ADDR(985),(0)); // phase_1
          _wb_iface->poke32(U2_REG_SR_ADDR(986),(0)); // incr_1
          _wb_iface->poke32(U2_REG_SR_ADDR(980),0);
         //dds_set_finish
          _wb_iface->poke32(U2_REG_SR_ADDR(255),0xFF); //
    }
    void initial_dds_for_calibration_PAX8V7(){
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xFF);
        // setup dds
          _wb_iface->poke32(U2_REG_SR_ADDR(980),1);
          _wb_iface->poke32(U2_REG_SR_ADDR(981),(0x03ff<<16)|0x03ff); // scale_0
          _wb_iface->poke32(U2_REG_SR_ADDR(982),((0x0) <<16)|(0xffff>>2)); // phase_0
          _wb_iface->poke32(U2_REG_SR_ADDR(983),((32768>>5)<<16)|32768>>5); // incr_0
//             //_wb_iface->poke32(U2_REG_SR_ADDR(983),((0>>6)<<16)|0>>6); // incr_0
          _wb_iface->poke32(U2_REG_SR_ADDR(984),(0)); // scale_1
          _wb_iface->poke32(U2_REG_SR_ADDR(985),(0)); // phase_1
          _wb_iface->poke32(U2_REG_SR_ADDR(986),(0)); // incr_1
          _wb_iface->poke32(U2_REG_SR_ADDR(980),0);
         //dds_set_finish
          _wb_iface->poke32(U2_REG_SR_ADDR(255),0xFF); //
    }
    void init_for_calibration_GNS(double _TEST_FREQ,double SAMPLE_RATE,bool test_mode){
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);
        double RX_GAIN=6;
        if(_TEST_FREQ>=.5e6 && _TEST_FREQ<=2.8e6)
            RX_GAIN=0;
        if(test_mode){
            for (uint32_t i=0; i<vAD9361.size(); i++){

                // internal lo (set by default)
                if(i==3){
                    vAD9361[i]->set_active_chains(true,false,true,false);
                    vAD9361[i]->set_clk_mux_parent(0);
                    vAD9361[i]->tune("RX",_TEST_FREQ);
                    vAD9361[i]->set_agc("RX1",false);
                    vAD9361[i]->set_gain("RX1",RX_GAIN);
                    vAD9361[i]->output_digital_test_tone(false);
                    vAD9361[i]->set_iq_balance_auto("RX",true);
                }else {
                        vAD9361[i]->set_active_chains(true,false,false,false);
                        vAD9361[i]->tune("TX",_TEST_FREQ);
                        vAD9361[i]->set_gain("TX1",70);
                        vAD9361[i]->set_gain("TX2",0);
                        vAD9361[i]->set_clock_rate(SAMPLE_RATE);
                    }

            }
        } else {
            vAD9361[3]->set_active_chains(true,false,true,false);
            vAD9361[3]->set_clk_mux_parent(0);
            vAD9361[3]->tune("RX",_TEST_FREQ);
            vAD9361[3]->set_agc("RX1",false);
            vAD9361[3]->set_gain("RX1",RX_GAIN);
            vAD9361[3]->output_digital_test_tone(false);
            vAD9361[3]->set_iq_balance_auto("RX",true);
        }
            do_mcs();
            initial_dds_for_calibration();
    }
    void init_for_calibration_PAX8K7(double _TEST_FREQ,double SAMPLE_RATE,bool test_mode){
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);
        double RX_GAIN= 25;

        if(_TEST_FREQ<2.4e9)
            RX_GAIN -= 15;
        else if((_TEST_FREQ>2.7e9) && (_TEST_FREQ<4e9))
            RX_GAIN+=20;
        for (uint32_t i=0; i<vAD9361.size(); i++){
            vAD9361[i]->set_active_chains(false,false,true,true);
            if(test_mode){
                vAD9361[i]->tune("RX",_TEST_FREQ);
                vAD9361[i]->set_clock_rate(SAMPLE_RATE);
            }
            vAD9361[i]->set_agc("RX1",false);
            vAD9361[i]->set_agc("RX2",false);
            vAD9361[i]->set_gain("RX1",RX_GAIN);
            vAD9361[i]->set_gain("RX2",RX_GAIN);
            vAD9361[i]->set_iq_balance_auto("RX",false);
            if(i==2){ // PH
                vAD9361[i]->set_active_chains(true,true,true,true);
                vAD9361[i]->tune("TX",_TEST_FREQ);
                vAD9361[i]->set_gain("TX1",80);
                vAD9361[i]->set_gain("TX2",0);
                vAD9361[i]->output_digital_test_tone(false);
            }
        }
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xFF);
            do_mcs();
            if(_TEST_FREQ >= 2.5e9){
                 _wb_iface->poke32(U2_REG_SR_ADDR(SR_TX_SW),3);
            }
            rst_fifo();
            initial_dds_for_calibration();

    }

    void init_for_calibration_PAX8V7(double _TEST_FREQ,double SAMPLE_RATE,bool test_mode){
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);
        double RX_GAIN= 30;

        if(_TEST_FREQ<2.4e9)
            RX_GAIN -= 15;
        else if((_TEST_FREQ>2.7e9) && (_TEST_FREQ<4e9))
            RX_GAIN+=20;
        for (uint32_t i=0; i<vAD9361.size(); i++){
            vAD9361[i]->set_active_chains(false,false,true,true);
            if(test_mode){
                vAD9361[i]->tune("RX",_TEST_FREQ);
                vAD9361[i]->set_clock_rate(SAMPLE_RATE);
            }
            vAD9361[i]->set_agc("RX1",false);
            vAD9361[i]->set_agc("RX2",false);
            vAD9361[i]->set_gain("RX1",RX_GAIN);
            vAD9361[i]->set_gain("RX2",RX_GAIN);
            vAD9361[i]->set_iq_balance_auto("RX",false);
            if(i == 1){ // PH
                vAD9361[i]->set_active_chains(true,true,true,true);
                vAD9361[i]->tune("TX",_TEST_FREQ);
                vAD9361[i]->set_gain("TX1",0);
                vAD9361[i]->set_gain("TX2",80);
                vAD9361[i]->output_digital_test_tone(false);
            }
        }
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xFF);
            do_mcs();
            if(_TEST_FREQ >= 2.5e9){
                 _wb_iface->poke32(U2_REG_SR_ADDR(SR_TX_SW),3);
            }
            rst_fifo();
            initial_dds_for_calibration_PAX8V7();

    }

    void PAX8K7_rx_cal_mode(bool state){
        if (state)
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_RX_SW),1);
        else
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_RX_SW),3);
    }
    void PAX8V7_rx_cal_mode(bool state){
        if (state)
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_RX_SW),3);
        else
            _wb_iface->poke32(U2_REG_SR_ADDR(SR_RX_SW),1);
    }
    void tx_sw_ch_for_cal_pax8(uint8_t channel){
//        if (channel>7) throw pax::value_error("wrong channel selected");
        switch (channel) {
            case 0: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),3); break;
            case 1: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),2); break;
            case 2: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),1); break;
            case 3: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),0); break;
            case 4: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),7); break;
            case 5: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),6); break;
            case 6: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),5); break;
            case 7: _wb_iface->poke32(U2_REG_SR_ADDR(SR_CAL_SEL),4); break;
        default:
            break;
        }
    }



    void poke32(boost::uint32_t addr, boost::uint32_t data){
        _wb_iface->poke32(addr, data);
    }

    boost::uint32_t  peek32(boost::uint32_t addr){
        return _wb_iface->peek32(addr);
    }

    void poke16(boost::uint32_t addr, boost::uint16_t data){
        _wb_iface->poke16(addr, data);
    }

    boost::uint16_t peek16(boost::uint32_t addr){
        return _wb_iface->peek16(addr);
    }

    void sw_for_calibration(uint8_t channel){
        if(channel >= 8){
            throw pax::value_error("wrong channel fo ampelifire");
        }
        switch (channel) {
        case 0:
            channel = 3;
          break;
        case 1:
             channel = 2;
          break;
        case 2:
            channel = 1;
             break;
        case 3:
             channel = 0;
            break;
        case 4:
            channel = 7;
          break;
        case 5:
            channel = 6;
          break;
        case 6:
            channel = 5;
             break;
        case 7:
            channel = 4;
            break;
        default:
            break;
        }

        // armin comment this in 260499
                int write_reg =0;
                std::bitset<3> temp(channel);
        //        if(temp[0]==1)                              // set the GPIO for setting multiplexer
        //            write_reg |= (((1<<4)<<6) |(1<<4)) ;
        //        if(temp[1]==1)
        //            write_reg |=( (1<<2)<<6 | (1<<2) );
        //        if(temp[2]==1)
        //            write_reg |= ( (1<<0)<<6 | (1<<0) );
        //        write_reg |= (((1<<3)<<6) |(1<<3));
        //        write_reg |= (((0<<1)<<6) |(0<<1));

        // armin add this in 260499
                if(temp[0]==1)                              // set the GPIO for setting multiplexer
                    write_reg |= (1<<2) ;
                if(temp[1]==1)
                    write_reg |= (1<<3) ;
                if(temp[2]==1)
                    write_reg |= (1<<4) ;
                write_reg |=  (0<<0) ;
                write_reg |=  (1<<1) ;

                _wb_iface->poke32(U2_REG_GPIO_ADDR(1),write_reg);
    }

    void setup_tx_dds(bool paxmode){
        // setup dds
        if (paxmode){ // must be used with proper frequency offset between TX/RX
            _wb_iface->poke32(U2_REG_SR_ADDR(980),1);
            _wb_iface->poke32(U2_REG_SR_ADDR(981),(0x3fff<<16)|0x3fff); // scale_0
            _wb_iface->poke32(U2_REG_SR_ADDR(982),(0<<16)|16384); // phase_0
            _wb_iface->poke32(U2_REG_SR_ADDR(983),((30719)<<16)|30719); // incr_0
            _wb_iface->poke32(U2_REG_SR_ADDR(984),(0<<16)|0); // scale_1
            _wb_iface->poke32(U2_REG_SR_ADDR(985),(0<<16)|0); // phase_1
            _wb_iface->poke32(U2_REG_SR_ADDR(986),(0<<16)|0); // incr_1
            _wb_iface->poke32(U2_REG_SR_ADDR(980),0);
            // setup corr dds
            _wb_iface->poke32(U2_REG_SR_ADDR(28),0x3fff); // scale
            _wb_iface->poke32(U2_REG_SR_ADDR(29),14335); // increment (1.25/20)
        } else{ // used at the same TX/RX frequency
            _wb_iface->poke32(U2_REG_SR_ADDR(980),1);
            _wb_iface->poke32(U2_REG_SR_ADDR(981),(0x3fff<<16)|0x3fff); // scale_0
            _wb_iface->poke32(U2_REG_SR_ADDR(982),(0<<16)|16384); // phase_0
            //_wb_iface->poke32(U2_REG_SR_ADDR(983),((14335)<<16)|14335); // incr_0
            _wb_iface->poke32(U2_REG_SR_ADDR(983),((1024)<<16)|1024); // incr_0
            _wb_iface->poke32(U2_REG_SR_ADDR(984),(0<<16)|0); // scale_1
            _wb_iface->poke32(U2_REG_SR_ADDR(985),(0<<16)|0); // phase_1
            _wb_iface->poke32(U2_REG_SR_ADDR(986),(0<<16)|0); // incr_1
            _wb_iface->poke32(U2_REG_SR_ADDR(980),0);
            // setup corr dds
            _wb_iface->poke32(U2_REG_SR_ADDR(28),0x3fff); // scale
            _wb_iface->poke32(U2_REG_SR_ADDR(29),1024); // increment (1.25/20)
        }
        // TX Sync
        _wb_iface->poke32(0x44044,1);
    }
protected:
    bool check_phase_calibration_met(double phase ,double& I_delay,double& Q_delay){
        boost::this_thread::sleep(boost::posix_time::microseconds(200));
        read_I_Q_CIC_GNS(I_delay,Q_delay);
        if((std::abs(std::atan2(Q_delay,I_delay))) <= (double)((phase * pi)/_degree_180)){
            boost::this_thread::sleep(boost::posix_time::microseconds(200));
            read_I_Q_CIC_GNS(I_delay,Q_delay);
            return ((std::abs(std::atan2(Q_delay,I_delay))) <= (double)((phase * pi)/_degree_180));
        }
        return false;

    }
    bool check_phase_calibration_PAX8V7_met(double phase ,double& I_delay,double& Q_delay){
        boost::this_thread::sleep(boost::posix_time::microseconds(200));
        read_I_Q_CIC_PAX8V7(I_delay,Q_delay);
        if((std::abs(std::atan2(Q_delay,I_delay))) <= (double)((phase * pi)/_degree_180)){
            boost::this_thread::sleep(boost::posix_time::microseconds(200));
            read_I_Q_CIC_PAX8V7(I_delay,Q_delay);
            return ((std::abs(std::atan2(Q_delay,I_delay))) <= (double)((phase * pi)/_degree_180));
        }
        return false;

    }


    void rst_fifo(){
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_CALIBRATION_RST_FIFO),0x1); //
        _wb_iface->poke32(U2_REG_SR_ADDR(SR_CALIBRATION_RST_FIFO),0x0); //
    }

    const uint32_t gns_cal_extra_median = 5;
    const double gns_accurecy_of_phase_calibration_degree = 0.5;
    double pax8_accurecy_of_phase_calibration_degree = 2;// 0.5

    const double pi = 3.1415;
    const double _degree_180 = 180;
    /***********************************************************************
     * Private variables
     **********************************************************************/
    std::vector<pax::usrp::ad9361_ctrl::sptr> vAD9361;
    pax::wb_iface::sptr _wb_iface;
    std::vector<pax::filter_bank::sptr> filter;
};

/***********************************************************************
 * Public make function for pax synchronization
 **********************************************************************/
PAX_API pax::pax_sync::sptr pax::pax_sync::make(std::vector<pax::usrp::ad9361_ctrl::sptr>& vAD9361, pax::wb_iface::sptr wb_iface){
    return pax_sync::sptr(new pax_sync_impl(vAD9361, wb_iface));
}


