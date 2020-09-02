#include "filter_bank_imple.hpp"

////////////////////////////////////////
/////////////aseman 2///////////////////
////////////////////////////////////////


PAX_API pax::filter_bank::sptr  pax::filter_bank_aseman2::make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361,
                                                             pax_iface::sptr _wb_iface, 
                                                             spi_wb_iface::sptr _spi_iface, 
                                                             uint _which_ad9361_ic)
{
    return pax::filter_bank::sptr(new pax::filter_bank_aseman2(_vAD9361, _wb_iface, _spi_iface, _which_ad9361_ic));
}

pax::filter_bank_aseman2::filter_bank_aseman2(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361,
                                        pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface,
                                         uint _which_ad9361_ic
                                        )
{
    vAD9361 = _vAD9361;
    iface = _wb_iface;
    spi_iface = _spi_iface;
    which_ad9361_ic = _which_ad9361_ic;
}


void  pax::filter_bank_aseman2::set_filter_path(double freq ,std::string direction, bool set_ad9361 )
{

    if(freq <= 60e6){
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_40MHz);
    } else if( freq <= 130e6) {
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_100MHz);
    } else if (freq <= 264e6){
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_250MHz);
    } else if (freq <= 470e6){
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_450MHz);
    } else if (freq <= 1000e6){
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_900MHz);
    } else if (freq <=2400e6){
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_2200MHz);
    }else {
        set_filter_path_sky_v2(filter_bank_sky_v2_value::low_pass_6000MHz);
    }
    if(set_ad9361){
        vAD9361[0]->tune("RX",freq, false);
    }

    //this is fo ignore unused direction warning
    std::string direction1;
    direction1 = direction;
    //
}
void  pax::filter_bank_aseman2::filter_bank_init() {
    sky_v2_filter_pin_map[sky_v2_PIN_1]=SR_GPO_0;
    sky_v2_filter_pin_map[sky_v2_PIN_2]=SR_GPO_1;
    sky_v2_filter_pin_map[sky_v2_PIN_3]=SR_GPO_2;
    sky_v2_filter_pin_map[sky_v2_PIN_4]=SR_GPO_3;
}

void pax::filter_bank_aseman2::do_rx_attenuation(int val)
{
    // for ignore unused val warning
    int val1 = val;
    val1++;

}

void pax::filter_bank_aseman2::set_regulator_control_manualy_TX_channles(uint8_t channle)
{   // for ignore unused channle warning
    uint8_t channle1 = channle;
    channle1++;
    //
    return;

}

void pax::filter_bank_aseman2::set_regulator_control_manualy_RX_channles(uint8_t channle)
{
    // for ignore unused channle warning
        uint8_t channle1 = channle;
        channle1++;
    //
    return;
}

void pax::filter_bank_aseman2::disable_all_regulators(bool accept)
{
    // for ignore unused accept1 warning
        bool accept1 = accept;
        accept1 = 1 ;
   //
    return;
}

void pax::filter_bank_aseman2::test_path(bool in)
{
    // for ignore unused accept1 warning
        bool in1 = in;
        in1 = 1 ;
   //
    return;
}

void pax::filter_bank_aseman2::set_filter_path_sky_v2(pax::filter_bank_aseman2::filter_bank_sky_v2_value::FILTER_PATH_SKY_V2 in)
{
    uint32_t temp=0;
    switch (in) {
    case filter_bank_sky_v2_value::low_pass_40MHz :{
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
    case filter_bank_sky_v2_value::low_pass_100MHz:   {
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
    case filter_bank_sky_v2_value::low_pass_250MHz:   {
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
    case filter_bank_sky_v2_value::low_pass_450MHz:
    {
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
    case filter_bank_sky_v2_value::low_pass_900MHz:   {
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
        case filter_bank_sky_v2_value::low_pass_2200MHz:  {
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
        case filter_bank_sky_v2_value::low_pass_6000MHz:  {
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_1]);
        temp |= (1) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_2]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_3]);
        temp |= (0) << static_cast<uint32_t>(sky_v2_filter_pin_map[sky_v2_PIN_4]);
    }break;
    default:
        throw value_error("wrong filter bank air v2 input");
        break;
    }


    iface->poke32(U2_REG_SR_ADDR(sr_filt_sw),temp);
}

////////////////////////////////////////
//////////////HAND OFF /////////////////
////////////////////////////////////////


PAX_API pax::filter_bank::sptr  pax::filter_bank_handoff::make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, spi_wb_iface::sptr _spi_iface, uint _which_ad9361_ic){
    return pax::filter_bank::sptr(new pax::filter_bank_handoff(_vAD9361, _wb_iface , _spi_iface, _which_ad9361_ic));
}

pax::filter_bank_handoff::filter_bank_handoff(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        )
{
    vAD9361 = _vAD9361;
    iface = _wb_iface;
    spi_iface = _spi_iface;
    which_ad9361_ic = _which_ad9361_ic;
}


void  pax::filter_bank_handoff::set_filter_path(double freq ,std::string direction, bool set_ad9361 )
{
    if(freq <= 60e6){
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_40MHz);
    } else if( freq <= 130e6) {
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_100MHz);
    } else if (freq <= 264e6) {
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_250MHz);
    } else if (freq <= 470e6) {
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_450MHz);
    } else if (freq <= 1000e6){
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_900MHz);
    } else if (freq <=2400e6) {
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_2200MHz);
    } else {
        set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::low_pass_6000MHz);
    }
    if(set_ad9361){
        vAD9361[which_ad9361_ic]->tune("RX",freq, false);
    }
}

void pax::filter_bank_handoff::filter_bank_init() 
{
    hand_off_filter_pin_map[HAND_OFF_PIN_1]=ADGPO_0;
    hand_off_filter_pin_map[HAND_OFF_PIN_2]=ADGPO_1;
    hand_off_filter_pin_map[HAND_OFF_PIN_3]=ADGPO_2;
    hand_off_filter_pin_map[HAND_OFF_PIN_4]=ADGPO_3;
}

void pax::filter_bank_handoff::do_rx_attenuation(int val)
{
    // for ignore unused val1 warning
        int val1 = val;
        val1 = 1 ;
   //
        return;

}

void pax::filter_bank_handoff::set_regulator_control_manualy_TX_channles(uint8_t channle)
{
    // for ignore unused channle warning
        uint8_t channle1 = channle;
        channle1 = 1 ;
   //
    return;

}

void pax::filter_bank_handoff::set_regulator_control_manualy_RX_channles(uint8_t channle)
{
    // for ignore unused channle warning
        uint8_t channle1 = channle;
        channle1 = 1 ;
   //
    return;
}

void pax::filter_bank_handoff::disable_all_regulators(bool accept)
{
    // for ignore unused channle warning
        bool accept1 = accept;
        accept1 = 1 ;
   //
    return;
}

void pax::filter_bank_handoff::test_path(bool in)
{

    // for ignore unused channle warning
        bool in1 = in;
        in1 = 1 ;
   //
    return;
}


void pax::filter_bank_handoff::set_filter_path_hand_off(pax::filter_bank_handoff::filter_bank_hand_off_value::FILTER_PATH_HAND_OFF in)
{
    switch (in) {
    case filter_bank_hand_off_value::low_pass_40MHz :{
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],1);
    }break;
    case filter_bank_hand_off_value::low_pass_100MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],0);
    }break;
    case filter_bank_hand_off_value::low_pass_250MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],1);
    }break;
    case filter_bank_hand_off_value::low_pass_450MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],0);
    }break;
        case filter_bank_hand_off_value::low_pass_900MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],0);
    }break;
        case filter_bank_hand_off_value::low_pass_2200MHz:  {
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],0);
    }break;
        case filter_bank_hand_off_value::low_pass_6000MHz:  {
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_2],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(hand_off_filter_pin_map[HAND_OFF_PIN_4],0);
    }break;
//    default:
//        throw value_error("filter bank hand off input");
//        break;
    }
}

////////////////////////////////////////
/////////////// SIMULATOR //////////////
////////////////////////////////////////
uint8_t pax::filter_bank_simulator::keep_last_changes_GPOA_spi10 = 0x00;
uint8_t pax::filter_bank_simulator::keep_last_changes_GPOA_spi11 = 0x00;
uint8_t pax::filter_bank_simulator::keep_last_changes_GPOB_spi10 = 0x00;
uint8_t pax::filter_bank_simulator::keep_last_changes_GPOB_spi11 = 0x00;

PAX_API pax::filter_bank::sptr  pax::filter_bank_simulator::make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic){
    return pax::filter_bank::sptr(new pax::filter_bank_simulator(_vAD9361,_wb_iface, _spi_iface, _which_ad9361_ic));
}

pax::filter_bank_simulator::filter_bank_simulator(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        )
{
    vAD9361 = _vAD9361;
    iface = _wb_iface;
    spi_iface = _spi_iface;
    which_ad9361_ic = _which_ad9361_ic;
}

bool pax::filter_bank_simulator::have_initiated_simulator = false;//////////////////////////////////////////////////////////////////
pax::filter_bank_simulator::GPO_A_B_STAT pax::filter_bank_simulator::sim_flt_status;
pax::spi_config_t pax::filter_bank_simulator::conf;

void  pax::filter_bank_simulator::set_filter_path(double freq ,std::string direction, bool set_ad9361 )
{
    _get_direction_from_antenna(direction);

    if(direction_t == RX){
        switch (which_ad9361_ic) {
            case 1:
            case 3:
            if(!activatedRegulator)
            {
                //std::cout << "RX ch= " << std::dec << which_ad9361_ic << " !activatedRegulator= " << !activatedRegulator << std::endl;
                // printf("RX ch= %d !activatedRegulator=%d \n",which_ad9361_ic,!activatedRegulator);
                keep_last_changes_GPOB_spi10 &= 0xBF;
                keep_last_changes_GPOB_spi10 |= 0x40; // spi_10 - GPB6
                spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);
                sim_flt_status.GPOB_CTRL_SW_TX &= 0xBF;
                sim_flt_status.GPOB_CTRL_SW_TX |= 0x40 ; //spi_11 - GPB6
                spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);
                set_rx_path_throw_outside_flt(false);
                activatedRegulator = true ;
            }
            else
            {
               // std::cout << "RX ch= " << std::dec << which_ad9361_ic << " activatedRegulator= " << activatedRegulator << std::endl;
                // printf("RX ch= %d activatedRegulator=%d \n",which_ad9361_ic,activatedRegulator);
                keep_last_changes_GPOB_spi10 |= 0xBF; //0x10111111; // spi_10 - GPB6
                spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);
                sim_flt_status.GPOB_CTRL_SW_TX |= 0xBF; //0x10111111 ; //spi_11 - GPB6
                spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);
                set_rx_path_throw_outside_flt(false);
                activatedRegulator = false ;

            }
            break;

            case 5:
            case 7:
            if(!activatedRegulator)
            {
                //std::cout << "RX ch= " << std::dec << which_ad9361_ic << " !activatedRegulator= " << !activatedRegulator << std::endl;
                // printf("RX ch= %d !activatedRegulator=%d \n",which_ad9361_ic,!activatedRegulator);
                keep_last_changes_GPOB_spi10 &= 0xDF;
                keep_last_changes_GPOB_spi10 |= 0x20 ;//0x20; // spi_10 - GPB5
                spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);
                keep_last_changes_GPOB_spi10 &= 0xEF;
                keep_last_changes_GPOB_spi10 |= 0x10; // spi_10 - GPB4
                spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);
                set_rx_path_throw_outside_flt(false);
                activatedRegulator = true ;
            }
            else
            {
                //std::cout << "RX ch= " << std::dec << which_ad9361_ic << " activatedRegulator= " << activatedRegulator << std::endl;
                // printf("RX ch= %d activatedRegulator=%d \n",which_ad9361_ic,activatedRegulator);
                keep_last_changes_GPOB_spi10 |= 0xFD; // spi_10 - GPB5
                spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);
                keep_last_changes_GPOB_spi10 |= 0xFE; // spi_10 - GPB4
                spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);
                set_rx_path_throw_outside_flt(false);
                activatedRegulator = false ;

            }
                break;
        default:
            break;
    }

    }else if(direction_t == TX){
        switch (which_ad9361_ic) {
            case 0:
            case 1:
                sim_flt_status.GPOA_FLT_OUTSIDE &= 0xFD;
                sim_flt_status.GPOA_FLT_OUTSIDE |= 0x02;  //keep_last_changes_GPOA_spi11
                spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_FLT_OUTSIDE, 24);// GPA1
                break;
            case 2:
            case 3:   //GPOB_CTRL_SW_TX
                sim_flt_status.GPOB_CTRL_SW_TX &= 0x7F;
                sim_flt_status.GPOB_CTRL_SW_TX |= 0x80;  //keep_last_changes_GPOA_spi11
                spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);// GPA1
                break;
            case 4:
            case 5:
                sim_flt_status.GPOA_FLT_OUTSIDE &= 0xFB;
                sim_flt_status.GPOA_FLT_OUTSIDE |= 0x04;  //keep_last_changes_GPOA_spi11
                spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_FLT_OUTSIDE, 24);// GPA1
                break;
            case 6:
            case 7:
               keep_last_changes_GPOB_spi10 &= 0x7F;
               keep_last_changes_GPOB_spi10 |= 0x80;
               spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24); //GPB7 in spi_10
               break;

        default:
            break;
    }
}

    if(direction_t == RX && freq != 0.00 ){
        switch (which_ad9361_ic) {
            case 1:
            case 3:
                if(freq <= 60e6){
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_40MHz);
                } else if( freq <= 130e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_100MHz);
                } else if (freq <= 264e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_250MHz);
                } else if (freq <= 470e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_450MHz);
                } else if (freq <= 1000e6){
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_900MHz);
                } else if (freq <=2400e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_2200MHz);
                } else {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_30_6000MHz::low_pass_6000MHz);
                }
                break;
            case 5:
            case 7:
                if(freq <= 1500e6){
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_1000_6000MHz::LFCN_1500MHz_HFCN_800MHz);
                } else if( freq <= 2250e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_1000_6000MHz::LFCN_2250MHz_HFCN_1500MHz);
                } else if (freq <= 3400e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_1000_6000MHz::LFCN3400MHz_HFCN2250MHz);
                } else if (freq <= 5850e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_1000_6000MHz::LFCN_5850MHz_HFCN_3100MHz);
                }
                break;
            default:
                break;
        }
    }else if(direction_t == TX && freq != 0.0 ){
        switch (which_ad9361_ic) {
            case 4:
            case 5:
                if (freq <= 3400e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_500_2500MHz::low_pass_900MHz);
                } else {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_500_2500MHz::low_pass_2200MHz);
                }
                break;
            case 2:
            case 3:
                if( freq <= 130e6) {
                   set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_100_1000MHz::low_pass_100MHz);
                } else if (freq <= 264e6) {
                   set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_100_1000MHz::low_pass_250MHz);
                } else if (freq <= 470e6) {
                   set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_100_1000MHz::low_pass_450MHz);
                } else {
                   set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_100_1000MHz::low_pass_900MHz);
                }
                break;
            case 0:
            case 1:
                if (freq <= 1000e6) {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_2000_6000MHz::low_pass_900MHz);
                } else {
                    set_sub_filter_path_simulator(filter_bank_sim_value::FILTER_PATH_2000_6000MHz::low_pass_2200MHz);
                }
                break;
            default:
                break;
        }
    }
    if(set_ad9361){
        vAD9361[which_ad9361_ic]->tune(direction,freq, false);
    }///what are u doing man ? :/

}


void pax::filter_bank_simulator::_get_direction_from_antenna(const std::string direction)
{
    std::string sub = direction.substr(0, 2);
    if (sub == "RX") {
        direction_t = RX;
    } else if (sub == "TX") {
        direction_t = TX;
    } else {
        direction_t = RX;
        throw pax::runtime_error("filter_bank got an invalid channel string.");
    }
}

void pax::filter_bank_simulator::set_rx_path_throw_outside_flt(bool throw_outside_amp){
    switch (which_ad9361_ic) {
    case 1:
    case 3:
        sim_flt_status.GPOA_FLT_OUTSIDE |= 0x11;
        sim_flt_status.GPOA_FLT_OUTSIDE &= ~(1 << 7);
        sim_flt_status.GPOA_FLT_OUTSIDE |= !throw_outside_amp << 7;
        break;
    case 5:
    case 7:
        sim_flt_status.GPOA_FLT_OUTSIDE |= 0x11;
        sim_flt_status.GPOA_FLT_OUTSIDE &= ~(1 << 3);
        sim_flt_status.GPOA_FLT_OUTSIDE |= !throw_outside_amp << 3;
        break;
    default:
        break;
    }

    spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_FLT_OUTSIDE, 24);
}


void pax::filter_bank_simulator::do_rx_attenuation(int value){
    uint8_t valueUnsigned8bit = static_cast<uint8_t> (value);
#ifdef __HAND_OFF__
    if(valueUnsigned8bit > 31)
        valueUnsigned8bit = 31;

    valueUnsigned8bit = valueUnsigned8bit << 1;
    valueUnsigned8bit ^= 0x3f;
    switch (which_ad9361_ic) {
    case 0:
        spi_iface->write_spi((1 << 2), conf, valueUnsigned8bit, 6);
        break;
    case 1:
        spi_iface->write_spi((1 << 3), conf, valueUnsigned8bit, 6);
        break;
    default:
        break;
    }

#else


    if(valueUnsigned8bit > 45)
        valueUnsigned8bit = 45;

    valueUnsigned8bit /= 3;
    valueUnsigned8bit ^= 0xf;
    switch (which_ad9361_ic) {
    case 1:
    case 3:
        spi_iface->write_spi((1 << 13), conf, valueUnsigned8bit, 4);
        break;
    case 5:
    case 7:
        spi_iface->write_spi((1 << 12), conf, valueUnsigned8bit, 4);
        break;
    default:
        break;
    }

#endif
}

void pax::filter_bank_simulator::test_path(bool in)
{
    switch (which_ad9361_ic){
    case 1:
    case 3:
    set_rx_path_throw_outside_flt(in); // path to out
        break;
    case 5:
    case 7:
//    set_rx_path_throw_outside_flt(in); // path to out
        if(in)
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | 0x00 , 24);
        else
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | 0xff , 24);
        break;

    default:
        break;

    }

}

void  pax::filter_bank_simulator::filter_bank_init() 
{
    if (!have_initiated_simulator){
        have_initiated_simulator = true;
        conf.mosi_edge = conf.EDGE_RISE;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x0A << 8) | 0x24, 24); // work in byte mode and bank 0
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x00 << 8) | 0x00, 24);
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x01 << 8) | 0x00, 24);
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x0C << 8) | 0xFF, 24);
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x0D << 8) | 0xFF, 24);
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x0A << 8) | 0x24, 24);
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x00 << 8) | 0x00, 24);
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x01 << 8) | 0x00, 24);
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x0C << 8) | 0xFF, 24);
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x0D << 8) | 0xFF, 24);
    }

    set_rx_path_throw_outside_flt(false); // path to ad9364
    disable_all_regulators(true);

    do_rx_attenuation(0);

}



void pax::filter_bank_simulator::set_sub_filter_path_simulator(pax::filter_bank_simulator::filter_bank_sim_value::FILTER_PATH_30_6000MHz::sub_flt in){
    std::vector<uint8_t> vec = {0x3, 0x0, 0x1, 0x2, 0xc, 0x8, 0x4};
    uint8_t last = sim_flt_status.GPOA_CTRL_SW_RX;

    switch (which_ad9361_ic) {
    case 1:
    case 3:
        sim_flt_status.GPOA_CTRL_SW_RX &= 0xf0;
        sim_flt_status.GPOA_CTRL_SW_RX |= vec[static_cast<int>(in)] << 0;
        break;
    case 5:
    case 7:
        sim_flt_status.GPOA_CTRL_SW_RX &= 0x0f;
        sim_flt_status.GPOA_CTRL_SW_RX |= vec[static_cast<int>(in)] << 4;
        break;
    default:
        break;
    }
    bool can_write =false;
    if(last != sim_flt_status.GPOA_CTRL_SW_RX){
    spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_CTRL_SW_RX, 24);
    can_write =true;
    }
}

void pax::filter_bank_simulator::set_sub_filter_path_simulator(pax::filter_bank_simulator::filter_bank_sim_value::FILTER_PATH_100_1000MHz::sub_flt in){
    std::vector<uint8_t> vec = {0x0, 0x1, 0x2, 0x3};
    uint8_t last = sim_flt_status.GPOB_CTRL_SW_TX;
    sim_flt_status.GPOB_CTRL_SW_TX &= 0xf3;
    sim_flt_status.GPOB_CTRL_SW_TX |= vec[static_cast<int>(in)] << 2;

    bool can_write =false;
    if(last != sim_flt_status.GPOB_CTRL_SW_TX)
    {
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);
        can_write = true;
    }

}

void pax::filter_bank_simulator::set_sub_filter_path_simulator(pax::filter_bank_simulator::filter_bank_sim_value::FILTER_PATH_500_2500MHz::sub_flt in){
    std::vector<uint8_t> vec = {0x1, 0x2};
    uint8_t last = sim_flt_status.GPOB_CTRL_SW_TX;
    sim_flt_status.GPOB_CTRL_SW_TX &= 0xfc;
    sim_flt_status.GPOB_CTRL_SW_TX |= vec[static_cast<int>(in)] << 0;

    bool can_write =false;
    if(last != sim_flt_status.GPOB_CTRL_SW_TX){
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);
        can_write =true;
    }
}

void pax::filter_bank_simulator::set_sub_filter_path_simulator(pax::filter_bank_simulator::filter_bank_sim_value::FILTER_PATH_2000_6000MHz::sub_flt in){
    std::vector<uint8_t> vec = {0x1, 0x2};
    uint8_t last = sim_flt_status.GPOB_CTRL_SW_TX;
    sim_flt_status.GPOB_CTRL_SW_TX &= 0xcf;
    sim_flt_status.GPOB_CTRL_SW_TX |= vec[static_cast<int>(in)] << 4;

    bool can_write =false;
    if(last != sim_flt_status.GPOB_CTRL_SW_TX){
     spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);
     can_write =true;
    }
}

void pax::filter_bank_simulator::set_sub_filter_path_simulator(pax::filter_bank_simulator::filter_bank_sim_value::FILTER_PATH_1000_6000MHz::sub_flt in){
    std::vector<uint8_t> vec = {0x0, 0x1 , 0x2 , 0x3};
    uint8_t last = sim_flt_status.GPOA_CTRL_SW_RX;

    sim_flt_status.GPOA_CTRL_SW_RX &= 0x3f; // 0011_1111
    sim_flt_status.GPOA_CTRL_SW_RX |= vec[static_cast<int>(in)] << 6;

    bool can_write =false;
    if(last != sim_flt_status.GPOA_CTRL_SW_RX){
    spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_CTRL_SW_RX, 24);
    can_write =true;
    }
}



void pax::filter_bank_simulator::set_regulator_control_manualy_TX_channles(uint8_t channle)
{

    conf.mosi_edge = conf.EDGE_RISE;
    switch (channle) {
    case 0:
    case 1:
       sim_flt_status.GPOA_FLT_OUTSIDE &= 0xFD; // keep all bits
       sim_flt_status.GPOA_FLT_OUTSIDE |= 0x02;  //keep_last_changes_GPOA_spi11
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_FLT_OUTSIDE, 24);// GPA1
       // keep_last_changes_GPOB = 0x01; // we must kept our last changes in gpio beacous 8 channle can change gpo value
        break;
    case 2:
    case 3:
        sim_flt_status.GPOB_CTRL_SW_TX &= 0x7F ;
        sim_flt_status.GPOB_CTRL_SW_TX |= 0x80 ; //keep_last_changes_GPOB_spi11
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24); // GPB7 in spi_11 => 1000_0000 = 0x40
        break;
    case 4:
    case 5:
        sim_flt_status.GPOA_FLT_OUTSIDE |= 0xFB;
        sim_flt_status.GPOA_FLT_OUTSIDE |= 0x04;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | sim_flt_status.GPOA_FLT_OUTSIDE, 24); // GPA2
        break;
    case 6:
    case 7:
        keep_last_changes_GPOB_spi10 |= 0x7F;
        keep_last_changes_GPOB_spi10 |= 0x80;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24); //GPB7 in spi_10
        break;
    default:
        break;
    }
}

void pax::filter_bank_simulator::set_regulator_control_manualy_RX_channles(uint8_t channle)
{

    conf.mosi_edge = conf.EDGE_RISE;
    switch (channle) {
    case 1:
    case 3:
        sim_flt_status.GPOB_CTRL_SW_TX &= 0xBF ;
        sim_flt_status.GPOB_CTRL_SW_TX |= 0x40 ;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | sim_flt_status.GPOB_CTRL_SW_TX, 24);// GPB6
        keep_last_changes_GPOB_spi10 &= 0xBF;
        keep_last_changes_GPOB_spi10 |= 0x40;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24);// GPB6
        break;
    case 5:
    case 7:
        keep_last_changes_GPOB_spi10 &= 0xDF;
        keep_last_changes_GPOB_spi10 |= 0x20;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24); // GPB5
        keep_last_changes_GPOB_spi10 &= 0xEF;
        keep_last_changes_GPOB_spi10 |= 0x10;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10, 24); // GPB4
        break;
    default:
        break;
    }

}

void pax::filter_bank_simulator::disable_all_regulators(bool accept)
{

    conf.mosi_edge = conf.EDGE_RISE;
    if(accept){

        keep_last_changes_GPOB_spi10 &= ~0x10;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB4
        keep_last_changes_GPOB_spi10 &= ~0x20;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10  , 24); //GPB5
        keep_last_changes_GPOB_spi10 &= ~0x40;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB6
        keep_last_changes_GPOB_spi10 &= ~0x80;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB7

        sim_flt_status.GPOB_CTRL_SW_TX &= ~0x40;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | (sim_flt_status.GPOB_CTRL_SW_TX), 24); // GPB6
        sim_flt_status.GPOB_CTRL_SW_TX &= ~0x80;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | (sim_flt_status.GPOB_CTRL_SW_TX), 24); // GPB7 in spi_11 => 1000_0000
        sim_flt_status.GPOA_FLT_OUTSIDE &= ~0x02;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | (sim_flt_status.GPOA_FLT_OUTSIDE), 24); // GPA2
        sim_flt_status.GPOA_FLT_OUTSIDE &= ~0x04;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | (sim_flt_status.GPOA_FLT_OUTSIDE), 24); // GPA1
    }
    else
    {

        keep_last_changes_GPOB_spi10 |= 0x10;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB4
        keep_last_changes_GPOB_spi10 |= 0x20;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB5
        keep_last_changes_GPOB_spi10 |= 0x40;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB6
        keep_last_changes_GPOB_spi10 |= 0x80;
        spi_iface->write_spi((1 << 10), conf, ((1 << 6) << 16) | (0x13 << 8) | keep_last_changes_GPOB_spi10 , 24); //GPB7
        
        sim_flt_status.GPOB_CTRL_SW_TX |= 0x40;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | (sim_flt_status.GPOB_CTRL_SW_TX), 24); // GPB6
        sim_flt_status.GPOB_CTRL_SW_TX |= 0x80;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x13 << 8) | (sim_flt_status.GPOB_CTRL_SW_TX), 24); // GPB7 in spi_11 => 1000_0000
        sim_flt_status.GPOA_FLT_OUTSIDE |= 0x02;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | (sim_flt_status.GPOA_FLT_OUTSIDE), 24); // GPA2
        sim_flt_status.GPOA_FLT_OUTSIDE |= 0x04;
        spi_iface->write_spi((1 << 11), conf, ((1 << 6) << 16) | (0x12 << 8) | (sim_flt_status.GPOA_FLT_OUTSIDE), 24); // GPA1
    }



}




////////////////////////////////////////
//////////////// VIRTEX ////////////////
////////////////////////////////////////


PAX_API pax::filter_bank::sptr  pax::filter_bank_virtex::make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic){
    return pax::filter_bank::sptr(new pax::filter_bank_virtex(_vAD9361,_wb_iface, _spi_iface, _which_ad9361_ic));
}

pax::filter_bank_virtex::filter_bank_virtex(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        )
{
    vAD9361 = _vAD9361;
    iface = _wb_iface;
    spi_iface = _spi_iface;
    which_ad9361_ic = _which_ad9361_ic;
};


void  pax::filter_bank_virtex::set_filter_path(double freq ,std::string direction, bool set_ad9361 )
{
    if(freq <= 60e6){
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_40MHz);
    } else if( freq <= 130e6) {
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_100MHz);
    } else if (freq <= 264e6) {
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_250MHz);
    } else if (freq <= 470e6) {
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_450MHz);
    } else if (freq <= 1000e6){
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_900MHz);
    } else if (freq <=2400e6) {
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_2200MHz);
    } else {
        set_filter_path_virtex(filter_bank_virtex_value::low_pass_6000MHz);
    }
    if(set_ad9361){
        vAD9361[which_ad9361_ic]->tune("RX",freq, false);
    }

    // for ignore direction unused warning
    std::string direction1;
    direction1 = direction;

}
void  pax::filter_bank_virtex::filter_bank_init() {
        virtex_filter_pin_map[VIRTEX_PIN_1]=VADGPO_0;
        virtex_filter_pin_map[VIRTEX_PIN_2]=VADGPO_1;
        virtex_filter_pin_map[VIRTEX_PIN_3]=VADGPO_2;
        virtex_filter_pin_map[VIRTEX_PIN_4]=VADGPO_3;
}

void pax::filter_bank_virtex::do_rx_attenuation(int val)
{
    // for ignore val unused warning
    int  val1;
    val1 = val;
    return;

}

void pax::filter_bank_virtex::set_regulator_control_manualy_TX_channles(uint8_t channle)
{
    // for ignore channle unused warning
    uint8_t  channle1;
    channle1 = channle;
    return;
}

void pax::filter_bank_virtex::set_regulator_control_manualy_RX_channles(uint8_t channle)
{
    // for ignore channle unused warning
    uint8_t  channle1;
    channle1 = channle;
    return;
}

void pax::filter_bank_virtex::disable_all_regulators(bool accept)
{
    // for ignore accept unused warning
    bool  accept1;
    accept1 = accept;
    return;
}

void pax::filter_bank_virtex::test_path(bool in)
{
    // for ignore channle unused warning
    bool  in1;
    in1 = in;
    return;
}
void pax::filter_bank_virtex::set_filter_path_virtex(pax::filter_bank_virtex::filter_bank_virtex_value::FILTER_PATH_virtex in)
{
    switch (in) {
    case filter_bank_virtex_value::low_pass_40MHz :{
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],1);
    }break;
    case filter_bank_virtex_value::low_pass_100MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],0);
    }break;
    case filter_bank_virtex_value::low_pass_250MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],1);
    }break;
    case filter_bank_virtex_value::low_pass_450MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],0);
    }break;
        case filter_bank_virtex_value::low_pass_900MHz:   {
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],0);
    }break;
        case filter_bank_virtex_value::low_pass_2200MHz:  {
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],0);
    }break;
        case filter_bank_virtex_value::low_pass_6000MHz:  {
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_1],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_2],1);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_3],0);
        vAD9361[which_ad9361_ic]->GPO_manual_set(virtex_filter_pin_map[VIRTEX_PIN_4],0);
    }break;
//    default:
//        throw value_error("filter bank virtex input");
//        break;
    }
}


////////////////////////////////////////
//////////////// NULLPTR ///////////////
////////////////////////////////////////
PAX_API pax::filter_bank::sptr  pax::filter_bank_nullptr::make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic){
    return pax::filter_bank::sptr(new pax::filter_bank_nullptr(_vAD9361,_wb_iface, _spi_iface, _which_ad9361_ic));
}

pax::filter_bank_nullptr::filter_bank_nullptr(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface,
                                        spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic
                                        )
{
    vAD9361 = _vAD9361;
    iface = _wb_iface;
    spi_iface = _spi_iface;
    which_ad9361_ic = _which_ad9361_ic;
};

void  pax::filter_bank_nullptr::filter_bank_init() 
{
    return;
}

void pax::filter_bank_nullptr::do_rx_attenuation(int val)
{
    // for ignore val unused warning
    int  val1;
    val1 = val;
    return;
}

void pax::filter_bank_nullptr::set_regulator_control_manualy_TX_channles(uint8_t channle)
{
    // for ignore channle unused warning
    uint8_t  channle1;
    channle1 = channle;
    return;
}

void pax::filter_bank_nullptr::set_regulator_control_manualy_RX_channles(uint8_t channle)
{
    // for ignore channle unused warning
    uint8_t  channle1;
    channle1 = channle;
    return;
}

void pax::filter_bank_nullptr::disable_all_regulators(bool accept)
{
    // for ignore accept unused warning
    bool  accept1;
    accept1 = accept;
    return;
}

void pax::filter_bank_nullptr::test_path(bool in)
{
    // for ignore in unused warning
    bool  in1;
    in1 = in;
    return;
}


void  pax::filter_bank_nullptr::set_filter_path(double freq ,std::string direction, bool set_ad9361 )
{
    // for ignore in unused warning
    double freq1;
    std::string direction1;
    bool set_ad9361_;
    freq1 = freq;
    direction1 = direction;
    set_ad9361_ = set_ad9361;
    return;
}
