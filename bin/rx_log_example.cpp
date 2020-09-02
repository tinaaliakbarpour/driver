 #include "recorder.hpp"

int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;
#define _SAMP_FREQ (40e6)
#define _TUNE_FREQ (800e6)

    mb_container_type tester;
    vec_streamers_t streamers=pax_init(tester,1);

    // disable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);
    double ADSampleRate;
    for (size_t i=0; i<tester.ad_9361.size(); i++)
    {
        tester.ad_9361[i]->set_active_chains(false,false,true,true);
        tester.ad_9361[i]->tune("RX",_TUNE_FREQ);
        tester.ad_9361[i]->set_agc("RX1",false);
        tester.ad_9361[i]->set_agc("RX2",false);
        tester.ad_9361[i]->set_gain("RX1",30);
        tester.ad_9361[i]->set_gain("RX2",30);
        ADSampleRate=tester.ad_9361[i]->set_clock_rate(_SAMP_FREQ);
    }

    std::cout<<"real sample_rate:  "<<ADSampleRate<<std::endl;
    for (size_t i=0; i<streamers.size(); i++){
        tester.rx_dsps[i]->set_tick_rate(ADSampleRate);
        tester.rx_dsps[i]->set_link_rate(ADSampleRate);
        tester.rx_dsps[i]->set_freq(-5e6);
        tester.rx_dsps[i]->set_host_rate(1e6);
    }

    // MCS
    //tester.sync->do_mcs();

    // enable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0xff);

    tester.sync->PAX8V7_calibration(_TUNE_FREQ,_SAMP_FREQ,false);//1580.42e6
    //tester.sync->PAX8K7_calibration(_TUNE_FREQ,_SAMP_FREQ,false);//1580.42e6

//    for (size_t i=0; i<tester.ad_9361.size(); i++)
//    {
//        tester.ad_9361[i]->set_gain("RX1",30);
//        tester.ad_9361[i]->set_gain("RX2",30);
//    }

    {pax::stream_cmd_t stream_cmd(pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    // send log command
    size_t num_log = 16384;
    stream_cmd.num_samps = num_log;
    stream_cmd.stream_mode=pax::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    for (size_t i=0; i<1; i++)
        tester.rx_dsps[i]->issue_stream_command(stream_cmd);



    std::vector<boost::thread*> runners(streamers.size());
    std::vector<recorder::sptr> recorders(streamers.size());
    for (size_t i=0; i<streamers.size(); i++){
     recorders[i]=recorder::make(streamers[i],i, tester,num_log*2);
     runners[i]=new boost::thread(boost::bind(&recorder::run,recorders[i]));
    }


    for(size_t i=0; i<4; i++)
        runners[i]->join();

    stream_cmd.num_samps = 1;
    stream_cmd.time_spec = pax::time_spec_t();
    stream_cmd.stream_mode=pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
    for (size_t i=0; i<8; i++)
        tester.rx_dsps[i]->issue_stream_command(stream_cmd);

    return 0;}





/*

    {

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
        for (size_t i=0; i<8; i++)
            tester.rx_dsps[i]->issue_stream_command(stream_cmd);

        std::vector<boost::thread*> runners(streamers.size());
        std::vector<recorder::sptr> recorders(streamers.size());

        for (size_t i=0; i<8 ;i++){
            recorders[i]=recorder::make(streamers[i],i,200000);
            runners[i]=new boost::thread(boost::bind(&recorder::run,recorders[i]));

        }

        runners[0]->join();
        streamers.data();
        return 0;

    }
*/
}
