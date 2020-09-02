#include "recorder.hpp"
#include <chrono>

int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;

	mb_container_type tester;
    vec_streamers_t streamers=pax_init(tester,8);

    // disable clock from ad9361
    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);

    double ADSampleRate;

    for (size_t i=0; i<2; i++)
    {
        tester.ad_9361[i]->set_active_chains(true,false,true,false);
        tester.ad_9361[i]->tune("RX",104e6);
        tester.ad_9361[i]->set_gain("RX1",30);
        tester.ad_9361[i]->set_gain("RX2",30);

        ADSampleRate=tester.ad_9361[i]->set_clock_rate(40e6);

    }
    std::cout<<"real sample_rate:  "<<ADSampleRate<<std::endl;

    float freq = 101e6;
    while(true){
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            tester.ad_9361[0]->tune("TX",freq);
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            std::cout << duration <<std::endl;
            freq += 101e6;
            if(freq >= 6e9) freq = 101e6;


    }



	return 0;
}
