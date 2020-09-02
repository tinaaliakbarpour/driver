//
// Copyright 2010-2017 Parto
//

#ifndef INCLUDED_PAX_SYNC_HPP
#define INCLUDED_PAX_SYNC_HPP

#include <serial.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include <wb_iface.hpp>
#include <string>
#include <wb_iface.hpp>
#include <ad9361_ctrl.hpp>
#include <filter_bank.hpp>


namespace pax {

/*!
 * The pax synchronization class:
 * Provides a set of functions to implementation layer.
 * Including spi, peek, poke, control...
 */
class PAX_API pax_sync
{
public:
    typedef boost::shared_ptr<pax_sync> sptr;
    /*!
     * Make a new pax synchronization class.
     * \param ctrl_transport the udp transport object
     * \return a new usrp2 interface object
     */
    static sptr  make(std::vector<pax::usrp::ad9361_ctrl::sptr>& vAD9361, pax::wb_iface::sptr wb_iface);

        //! The list of possible revision types
    enum rev_type {
        PAX8_REV0 = 1,
//        USRP2_REV3 = 3,
//        USRP2_REV4 = 4,
//        USRP_N200 = 200,
//        USRP_N200_R4 = 201,
//        USRP_N210 = 210,
//        USRP_N210_R4 = 211,
//        USRP_NXXX = 0
        PAX_UNKNOWN = 0
    };

    //! Get the revision type for this device
    virtual rev_type get_rev(void) = 0;

    //! Do multichip synchronization (baseband pll calibration)
    virtual void do_mcs() = 0;


    virtual void PAX8K7_rx_cal_mode(bool state) = 0;
    virtual void PAX8V7_rx_cal_mode(bool state) = 0;
    virtual std::vector<double> PAX8K7_calibration(double _TEST_FREQ = 1575e6 ,double SAMPLE_RATE = 32e6,bool test_mode = true) = 0;
    virtual std::vector<double> PAX8V7_calibration(double _TEST_FREQ = 1575e6 ,double SAMPLE_RATE = 32e6,bool test_mode = true) = 0;
    virtual void PAX8K7_set_accuracy_value_of_phase_cal(double val) = 0;
    virtual double PAX8K7_get_accuracy_value_of_phase_cal() = 0;
    virtual double read_phase_loc_ant(int channel) = 0;

    virtual void set_filter_bank(std::vector<boost::shared_ptr<filter_bank>> flt) = 0; // PH



    virtual void GNS_pass_channel() = 0;
    virtual bool GNS_calibration(double freq = 1575e6,double sample_rate = 32e6,bool test_mode = false) = 0;

};

}

#endif /* INCLUDED_PAX_SYNC_HPP */
