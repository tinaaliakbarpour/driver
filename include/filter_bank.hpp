//
// Copyright 2010-2016 Parto
//

#ifndef INCLUDE_FILTER_BANK_IMPLE_HPP
#define INCLUDE_FILTER_BANK_IMPLE_HPP


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
#include <exception>



////
#include <serial.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include <wb_iface.hpp>
#include <string>
#include <wb_iface.hpp>
#include <ad9361_ctrl.hpp>
///
namespace pax {
    class PAX_API filter_bank{
    public : 
        typedef boost::shared_ptr<filter_bank> sptr;
        
        static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& _vAD9361, pax_iface::sptr _wb_iface, 
                          spi_wb_iface::sptr _spi_iface, uint8_t _which_ad9361_ic) ;


        virtual void set_filter_path(double freq ,std::string direction, bool set_ad9361 = true) = 0;
        virtual void filter_bank_init() = 0;
        virtual void do_rx_attenuation(int val) = 0;
        virtual void set_regulator_control_manualy_TX_channles(uint8_t channle)= 0; // armin add this
        virtual void set_regulator_control_manualy_RX_channles(uint8_t channle)= 0; // armin add this
        virtual void disable_all_regulators(bool accept = true)= 0;   // armin add this
        virtual void test_path(bool in) = 0; // armin add this


    protected :

    std::vector<pax::usrp::ad9361_ctrl::sptr> vAD9361;
    pax_iface::sptr iface;
    spi_wb_iface::sptr spi_iface;
    uint8_t which_ad9361_ic;

    };


}


#endif /* INCLUDED_PAX_SYNC_HPP */
