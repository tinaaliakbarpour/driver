#include "filter_bank.hpp"

////////////////////////////////////////
/////////////aseman 2///////////////////
////////////////////////////////////////

namespace pax{
class PAX_API filter_bank_aseman2 : public pax::filter_bank{
    
    typedef struct
    {
        typedef enum {low_pass_40MHz=0,low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz,low_pass_2200MHz,low_pass_6000MHz} FILTER_PATH_SKY_V2;
    }filter_bank_sky_v2_value;
    public:

        static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, 
                          spi_wb_iface::sptr _spi_iface, uint _which_ad9361_ic);

        void set_filter_path(double freq ,std::string direction, bool set_ad9361 = true) ;
        void filter_bank_init() ;
        void do_rx_attenuation(int val);
        void set_regulator_control_manualy_TX_channles(uint8_t channle); // armin add this
        void set_regulator_control_manualy_RX_channles(uint8_t channle); // armin add this
        void disable_all_regulators(bool accept = true);   // armin add this
        void test_path(bool in);// armin add this

    private:
        
        filter_bank_aseman2(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361,
                                        pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface,
                                         uint _which_ad9361_ic
                                        );

        typedef enum {SR_GPO_0=0,SR_GPO_1=1,SR_GPO_2=2,SR_GPO_3=3} SETTING_REG_GPO_PIN_NUM;
        typedef enum {sky_v2_PIN_1=0,sky_v2_PIN_2,sky_v2_PIN_3,sky_v2_PIN_4} sky_v2_PIN_NUMBER;
        pax::dict<sky_v2_PIN_NUMBER,SETTING_REG_GPO_PIN_NUM> sky_v2_filter_pin_map;
        const uint32_t sr_filt_sw = 1021;

        void set_filter_path_sky_v2(filter_bank_sky_v2_value::FILTER_PATH_SKY_V2 in);

    
};

////////////////////////////////////////
//////////////HAND OFF /////////////////
////////////////////////////////////////


class PAX_API filter_bank_handoff : public pax::filter_bank{
    
typedef struct
{
    typedef enum {low_pass_40MHz=0,low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz,low_pass_2200MHz,low_pass_6000MHz} FILTER_PATH_HAND_OFF;
}filter_bank_hand_off_value;


    public:

        static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, 
                          spi_wb_iface::sptr _spi_iface, uint _which_ad9361_ic);

        void set_filter_path(double freq ,std::string direction, bool set_ad9361 = true) ;
        void filter_bank_init() ;
        void do_rx_attenuation(int val);
        void set_regulator_control_manualy_TX_channles(uint8_t channle); // armin add this
        void set_regulator_control_manualy_RX_channles(uint8_t channle); // armin add this
        void disable_all_regulators(bool accept = true);   // armin add this
        void test_path(bool in);// armin add this

    private:
        
        filter_bank_handoff(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        );

        typedef enum {ADGPO_0=0,ADGPO_1,ADGPO_2,ADGPO_3} AD_GPO_NUM;
        typedef enum {HAND_OFF_PIN_1=0,HAND_OFF_PIN_2,HAND_OFF_PIN_3,HAND_OFF_PIN_4} HAND_OFF_PIN_NUMBER;
        pax::dict<HAND_OFF_PIN_NUMBER,AD_GPO_NUM> hand_off_filter_pin_map;


        void set_filter_path_hand_off(filter_bank_hand_off_value::FILTER_PATH_HAND_OFF in);

    
};

////////////////////////////////////////
/////////////// SIMULATOR //////////////
////////////////////////////////////////

class PAX_API filter_bank_simulator : public pax::filter_bank{
    
    typedef struct
    {
        typedef struct
        {
            typedef enum {low_pass_40MHz=0,low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz,low_pass_2200MHz,low_pass_6000MHz} sub_flt;
        }FILTER_PATH_30_6000MHz;

        typedef struct
        {
            typedef enum {LFCN_1500MHz_HFCN_800MHz=0 , LFCN_2250MHz_HFCN_1500MHz = 3 ,LFCN3400MHz_HFCN2250MHz = 2,LFCN_5850MHz_HFCN_3100MHz = 1} sub_flt;
        }FILTER_PATH_1000_6000MHz;

        typedef struct
        {
            typedef enum {low_pass_900MHz,low_pass_2200MHz} sub_flt;
        }FILTER_PATH_2000_6000MHz;

        typedef struct
        {
            typedef enum {low_pass_900MHz,low_pass_2200MHz} sub_flt;
        }FILTER_PATH_500_2500MHz;

        typedef struct
        {
            typedef enum {low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz} sub_flt;
        }FILTER_PATH_100_1000MHz;
    }filter_bank_sim_value;

    typedef struct {
        uint8_t GPOA_CTRL_SW_RX;
        uint8_t GPOB_ATT_PUP;
        uint8_t GPOA_FLT_OUTSIDE; // SPI_11 - GPOA
        uint8_t GPOB_CTRL_SW_TX;  // SPI_11 - GPOB
    }GPO_A_B_STAT;


    public:

        static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, 
                          spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic);

        void set_filter_path(double freq ,std::string direction, bool set_ad9361 = true) ;
        void filter_bank_init() ;
        void do_rx_attenuation(int val);
        static uint8_t keep_last_changes_GPOB_spi10;
        static uint8_t keep_last_changes_GPOB_spi11;
        static uint8_t keep_last_changes_GPOA_spi10;
        static uint8_t keep_last_changes_GPOA_spi11;
        void set_regulator_control_manualy_TX_channles(uint8_t channle); // armin add this
        void set_regulator_control_manualy_RX_channles(uint8_t channle); // armin add this
        void disable_all_regulators(bool accept = true);   // armin add this
        void test_path(bool in);// armin add this


    private:
        
        filter_bank_simulator(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        );

    enum { RX = 0, TX } direction_t;
    static GPO_A_B_STAT sim_flt_status;
    static spi_config_t conf;
    static bool have_initiated_simulator;

    bool activatedRegulator = false ;
    bool first_time_call_ch_1_3 = false;
    bool first_time_call_ch_5_7 = false;

  


    void set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::sub_flt in);
    void set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_1000_6000MHz::sub_flt in);
    void set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_100_1000MHz::sub_flt in);
    void set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_500_2500MHz::sub_flt in);
    void set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_2000_6000MHz::sub_flt in);
    void set_rx_path_throw_outside_flt(bool throw_outside_amp = false);
    void _get_direction_from_antenna(const std::string direction);

};



////////////////////////////////////////
//////////////// VIRTEX ////////////////
////////////////////////////////////////

class PAX_API filter_bank_virtex : public pax::filter_bank{
    
    typedef struct
    {
        typedef enum {low_pass_40MHz=0,low_pass_100MHz,low_pass_250MHz,low_pass_450MHz,low_pass_900MHz,low_pass_2200MHz,low_pass_6000MHz} FILTER_PATH_virtex;
    }filter_bank_virtex_value;


    public:

        static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, 
                          spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic);

        void set_filter_path(double freq ,std::string direction, bool set_ad9361 = true) ;
        void filter_bank_init() ;
        void do_rx_attenuation(int val);
        void set_regulator_control_manualy_TX_channles(uint8_t channle); // armin add this
        void set_regulator_control_manualy_RX_channles(uint8_t channle); // armin add this
        void disable_all_regulators(bool accept = true);   // armin add this
        void test_path(bool in);// armin add this

    private:
        
        filter_bank_virtex(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        );

        void set_filter_path_virtex(filter_bank_virtex_value::FILTER_PATH_virtex in);

        typedef enum {VADGPO_0=0,VADGPO_1,VADGPO_2,VADGPO_3} VAD_GPO_NUM;
        typedef enum {VIRTEX_PIN_1=0,VIRTEX_PIN_2,VIRTEX_PIN_3,VIRTEX_PIN_4} VIRTEX_PIN_NUMBER;
        pax::dict<VIRTEX_PIN_NUMBER,VAD_GPO_NUM> virtex_filter_pin_map;
    
};
////////////////////////////////////////
//////////////// NULLPTR ///////////////
////////////////////////////////////////

//basseri decision 
class PAX_API filter_bank_nullptr : public pax::filter_bank{
    
     public:

        static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, 
                          spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic);

        void set_filter_path(double freq ,std::string direction, bool set_ad9361 = true) ;
        void filter_bank_init() ;
        void do_rx_attenuation(int val);
        void set_regulator_control_manualy_TX_channles(uint8_t channle); // armin add this
        void set_regulator_control_manualy_RX_channles(uint8_t channle); // armin add this
        void disable_all_regulators(bool accept = true);   // armin add this
        void test_path(bool in);// armin add this

    private:
            filter_bank_nullptr(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        );
    
};


}
