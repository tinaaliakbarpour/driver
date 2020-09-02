#include "recorder.hpp"
#include <eeprom.hpp>

#include <chrono>


typedef struct {
    uint32_t header;
    uint32_t task_id;
    uint32_t num_sample;
    uint32_t   samples [65536];
} packet_frame_t;

int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;
    mb_container_type tester;
    vec_streamers_t streamers=pax_init(tester,8);


    // disable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(23), 0x0);
    tester.iface->poke32(U2_REG_SR_ADDR(24), 0x0);



    double ADSampleRate;
    for(uint32_t i=0;i<tester.ad_9361.size();i++)
    {
        tester.ad_9361[i]->set_active_chains(false,false,true,false);
        // internal lo (set by default)


        tester.ad_9361[i]->tune("RX",1000e6);


        tester.ad_9361[i]->set_agc("RX1",false);
      //  tester.ad_9361[_AD9361]->set_agc("RX2",false);
        tester.ad_9361[i]->set_gain("RX1",10);
        tester.ad_9361[i]->set_gain("RX2",10);

        ADSampleRate=tester.ad_9361[i]->set_clock_rate(50e6);

    }

    tester.iface->poke32(U2_REG_SR_ADDR(990), 0x01);
    tester.iface->poke32(U2_REG_SR_ADDR(991), 0x01);

    // enable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(24), 0x01);
   /* tester.iface->poke32(U2_REG_SR_ADDR(25), 0x01);
    tester.iface->poke32(U2_REG_SR_ADDR(26), 0x01);
*/
streamers[1]->flush_all();
    pax::rx_metadata_t md;
    int num_requested_samples=65536;//10000;    /// PH

    pax::stream_cmd_t stream_cmd((num_requested_samples == 0)?
                                     pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
                                     pax::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
                                     );

    stream_cmd.num_samps = 1;
    stream_cmd.stream_now = true;
    stream_cmd.stream_mode=pax::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
    stream_cmd.time_spec = pax::time_spec_t();

    //tester.rx_dsps[0]->issue_stream_command(stream_cmd);




//    for (size_t i=0; i<1; i++)
        tester.rx_dsps[1]->issue_stream_command(stream_cmd);


    uint32_t num_of_samples = 65536;
    tester.iface->poke32(U2_REG_SR_ADDR(91), num_of_samples);
    //tester.iface->poke32(U2_REG_SR_ADDR(990), 5);
    tester.iface->poke32(U2_REG_SR_ADDR(92), 0x012345678);
    tester.iface->poke32(U2_REG_SR_ADDR(94), 0x6);

//    tester.iface->poke32(U2_REG_SR_ADDR(90), 0x0);
//    tester.iface->poke32(U2_REG_SR_ADDR(93), 0x1);
//    tester.iface->poke32(U2_REG_SR_ADDR(93), 0x0);

    int counter = 0;
    size_t total = 0;


    packet_frame_t buffs;


    while(true){
//        tester.iface->poke32(U2_REG_SR_ADDR(24), 0x01);


        tester.iface->poke32(U2_REG_SR_ADDR(90), 0x1);


        total = streamers[1]->recv(&buffs, num_of_samples + 3, md, 1, false);
//        if((total != num_of_samples + 3) || buffs.header != 0xaaaaaaaa){
//            std::cout << "Loss!  ";
//            tester.iface->poke32(U2_REG_SR_ADDR(93), 0x1);
//            tester.iface->poke32(U2_REG_SR_ADDR(93), 0x0);
//            streamers[1]->recv(&buffs, sizeof(buffs), md, 0.1, false);
//            continue;
//        }

//        if(total != num_of_samples + 3) std::cout << "Er:" << total << "           " << counter++ << std::endl;
        std::cout <<std::hex<< buffs.header <<"    "<<std::hex<< buffs.num_sample<<"    " <<std::hex<< buffs.task_id << std::endl ;
//        std::cout << std::endl <<total<<"    "<<counter++ << std::endl;
    }


    std::cout<<"real sample_rate:  "<<ADSampleRate<<std::endl;

    return 0;
}
