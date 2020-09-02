#include <config.h>
#include "recorder.hpp"
#include <eeprom.hpp>


int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;


    vec_IP_t IPs=find_all_pax_ip();
    vec_pax_dev_t pax_devices;
    for(uint32_t i=0;i<IPs.size();i++){
        pax_dev_t tmp=pax_init(2,IPs[i]);
        pax_devices.push_back(tmp);
    }

     mb_container_type tester;
     vec_streamers_t  streamers;

     for(uint32_t i=0;i<pax_devices.size();i++){
        if(pax_devices[i].pax_dev_addr["addr"]=="192.168.10.3"){      //select which ip you want or you can select
            tester= pax_devices[i].dev;
            streamers=pax_devices[i].streamers;
        }
     }



    int _AD9361= 0;

    // disable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);

    double ADSampleRate;

    tester.ad_9361[_AD9361]->set_active_chains(false,false,true,true);
    // internal lo (set by default)
    tester.ad_9361[_AD9361]->set_clk_mux_parent(0);
  //  tester.ad_9361[_AD9361]->tune("TX",200e6);
    tester.ad_9361[_AD9361]->set_gain("TX1",89.75);
   // tester.ad_9361[_AD9361]->set_gain("TX2",89.75);
    tester.ad_9361[_AD9361]->tune("RX",99e6);
    tester.ad_9361[_AD9361]->set_agc("RX1",false);
  //  tester.ad_9361[_AD9361]->set_agc("RX2",false);

    tester.ad_9361[_AD9361]->set_gain("RX1",30);
    tester.ad_9361[_AD9361]->set_gain("RX2",0);

    ADSampleRate=tester.ad_9361[_AD9361]->set_clock_rate(256e3*100);

    std::cout<<"real sample_rate:  "<<ADSampleRate<<std::endl;
    for (size_t i=0; i<streamers.size(); i++){
        tester.rx_dsps[i]->set_tick_rate(ADSampleRate);
        tester.rx_dsps[i]->set_link_rate(ADSampleRate);
        tester.rx_dsps[i]->set_host_rate(256e3);
        tester.rx_dsps[i]->set_freq(-5e6);
    }

    // enable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x1);

    tester.iface->poke32(U2_REG_SR_ADDR(990), 0x01);
    tester.iface->poke32(U2_REG_SR_ADDR(991), 0x01);


    tester.ad_9361[0]->output_digital_test_tone(false);

    // setup dds
    tester.iface->poke32(U2_REG_SR_ADDR(980),1);
    tester.iface->poke32(U2_REG_SR_ADDR(981),(0x1fff<<16)|0x1fff); // scale_0
    tester.iface->poke32(U2_REG_SR_ADDR(982),(0<<16)|32768>>1); // phase_0
    tester.iface->poke32(U2_REG_SR_ADDR(983),((32768>>1)<<16)|32768>>1); // incr_0
    //tester.iface->poke32(U2_REG_SR_ADDR(983),((0>>6)<<16)|0>>6); // incr_0
    tester.iface->poke32(U2_REG_SR_ADDR(984),((0x1fff<<16)>>0)|(0x1fff>>0)); // scale_1
    tester.iface->poke32(U2_REG_SR_ADDR(985),(0<<16)|32768>>1); // phase_1
    tester.iface->poke32(U2_REG_SR_ADDR(986),(185*2)|185*2); // incr_1
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

    for (size_t i=0; i<streamers.size(); i++){
        recorders[i]=recorder::make(streamers[i],i, tester,200000);
        runners[i]=new boost::thread(boost::bind(&recorder::run,recorders[i]));
    }

    runners[0]->join();

    return 0;
}
