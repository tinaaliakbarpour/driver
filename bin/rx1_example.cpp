#include "recorder.hpp"
#include <eeprom.hpp>

#include <chrono>

int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;

    mb_container_type tester;
    vec_streamers_t streamers=pax_init(tester,2);


//    pax::flash_vec_t temp, temp1;
//    temp=tester.iface->read_uid_flash();

    // disable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);
    //
//    int a;
//    while(1) a = tester.fifo_ctrl->peek32(READBACK_BASE + 4*2);


    double ADSampleRate;
    for(uint32_t i=0;i<tester.ad_9361.size();i++)
    {
        tester.ad_9361[i]->set_active_chains(false,false,true,true);
        // internal lo (set by default)
        tester.ad_9361[i]->tune("RX",100e6);
        tester.ad_9361[i]->set_agc("RX1",false); // automatic gain control _ peak to be const
        tester.ad_9361[i]->set_gain("RX1",30);
        tester.ad_9361[i]->set_gain("RX2",30);

        ADSampleRate=tester.ad_9361[i]->set_clock_rate(40e6);

    }
    tester.sync->GNS_calibration();
//    tester.ad_9361[0]->tune("TX",100e6);
//    tester.ad_9361[0]->tune("TX",200e6);
//    tester.ad_9361[0]->tune("TX",300e6);
//    tester.ad_9361[0]->tune("TX",500e6);

    // enable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xFF);

//    int w=0;
//    while(1){
//        w++;
//        ADSampleRate=tester.ad_9361[0]->set_clock_rate(w*1e6);
//        if(w>60)
//            w=5;

//    }

    //tester.sync->PAX8V7_rx_cal_mode(false);
    //tester.sync->do_mcs();
    //tester.sync->PAX8V7_calibration(0,0,false);
    std::cout<<"real sample_rate:  "<<ADSampleRate<<std::endl;

    for (size_t i=0; i<streamers.size(); i++){
        tester.rx_dsps[i]->set_tick_rate(ADSampleRate);
        tester.rx_dsps[i]->set_link_rate(ADSampleRate);
        tester.rx_dsps[i]->set_host_rate(7e6);
        tester.rx_dsps[i]->set_freq(-5e6);
    }


    tester.iface->poke32(U2_REG_SR_ADDR(990), 0x01);
    tester.iface->poke32(U2_REG_SR_ADDR(991), 0x01);


    tester.ad_9361[0]->output_digital_test_tone(false);

    // setup dds
    tester.iface->poke32(U2_REG_SR_ADDR(980),1);
    tester.iface->poke32(U2_REG_SR_ADDR(981),(0x1fff<<16)|0x1fff); // scale_0
    tester.iface->poke32(U2_REG_SR_ADDR(982),(0<<16)|32768>>1); // phase_0
    tester.iface->poke32(U2_REG_SR_ADDR(983),((32768>>1)<<16)|32768>>1); // incr_0
    //tester.iface->poke32(U2_REG_SR_ADDR(983),((0>>6)<<16)|0>>6); // incr_0
    tester.iface->poke32(U2_REG_SR_ADDR(984),((0<<16)>>0)|(0>>0)); // scale_1
    tester.iface->poke32(U2_REG_SR_ADDR(985),(0<<16)|0>>1); // phase_1
    tester.iface->poke32(U2_REG_SR_ADDR(986),(0)|0); // incr_1
    tester.iface->poke32(U2_REG_SR_ADDR(980),0);

    // record
    pax::rx_metadata_t md;
    int num_requested_samples=10000;

    pax::stream_cmd_t stream_cmd((num_requested_samples == 0)?
                                     pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
                                     pax::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
                                     );

    stream_cmd.num_samps = 1;
    stream_cmd.stream_now = true;
    stream_cmd.stream_mode=pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
    stream_cmd.time_spec = pax::time_spec_t();

    //tester.rx_dsps[0]->issue_stream_command(stream_cmd);
    //tester.ad_9361[0]->set_clk_mux_parent(1);
    for (size_t i=0; i<2; i++)
        tester.rx_dsps[i]->issue_stream_command(stream_cmd);

    std::vector<boost::thread*> runners(streamers.size());
    std::vector<recorder::sptr> recorders(streamers.size());

    for (size_t i=0; i<2 ;i++){
        recorders[i]=recorder::make(streamers[i],i, tester,200000);
        runners[i]=new boost::thread(boost::bind(&recorder::run,recorders[i]));

    }

    runners[0]->join();
    streamers.data();
    return 0;
}
