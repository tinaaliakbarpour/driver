//
// Copyright 2014 Ettus Research LLC
//

#ifndef INCLUDED_AD9361_CLIENT_H
#define INCLUDED_AD9361_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <pax_iface.h>

namespace pax {class filter_bank; // PH
 namespace usrp {

/*!
 * Filter bank band settings
 */
typedef enum {
    NO_FILTER_BANK,
    FILTER_BANK_BAND0,
    FILTER_BANK_BAND1,
    FILTER_BANK_BAND2,
    FILTER_BANK_BAND3
} filterbank_band_t;

/*!
 * Frequency band settings
 */
typedef enum {
    AD9361_RX_BAND0_MAX_LIMIT,
    AD9361_RX_BAND1_MAX_LIMIT,
    AD9361_RX_BAND2_MAX_LIMIT,
    AD9361_TX_BAND0_MAX_LIMIT,
    AD9361_TX_BAND1_MAX_LIMIT
} frequency_band_t;

/*!
 * Clocking mode
 */
typedef enum {
    AD9361_XTAL_P_CLK_PATH,
    AD9361_XTAL_N_CLK_PATH
} clocking_mode_t;

/*!
 * Digital interface specific
 */
typedef enum {
    AD9361_DDR_FDD_LVCMOS,
    AD9361_DDR_FDD_LVDS,

    AD9361_DDR_TDD_LVCMOS_1R_1T,
    AD9361_DDR_FDD_LVCMOS_1R_1T,
    AD9361_DDR_FDD_LVDS_1R_1T,
    AD9361_DDR_FDD_LVCMOS_2R_2T,
    AD9361_DDR_FDD_LVDS_2R_2T,
    AD9361_DDR_FDD_LVDS_2R_2T_Virtex,
    AD9361_UNKOWN


} digital_interface_mode_t;

/*!
 * Interface timing
 */
typedef struct {
    boost::uint8_t rx_clk_delay;
    boost::uint8_t rx_data_delay;
    boost::uint8_t tx_clk_delay;
    boost::uint8_t tx_data_delay;
} digital_interface_delays_t;

typedef enum {
    Analog=0,
    pax8,
    pax2,
    pax2s6,
    pax8_gnss_4ch,
    pax8_gnss_8ch,
    pax8_gnss_8ch_calibrasion,
    pax8_gnss_8ch_monitoring,
    pax2_9361_filter_bank,
    pax8v7_9361_filter_bank,
    NULL_B
} daughter_t;

typedef enum {
    _20MHz = 20,
    _40MHz = 40
}ad_ref_clk_t;

typedef enum {
    pax8_D_K325T,
    pax8_D_K410T,
    pax2_D,
    pax2s6_D,
    pax8v7_D,
    NULL_D
} main_board_t;



class ad9361_params {
public:
    typedef boost::shared_ptr<ad9361_params> sptr;

    virtual ~ad9361_params() {}

    virtual digital_interface_delays_t get_digital_interface_timing(size_t which) = 0;
    virtual digital_interface_mode_t get_digital_interface_mode() = 0;
    virtual clocking_mode_t get_clocking_mode() = 0;
    virtual double get_band_edge(frequency_band_t band) = 0;
    virtual void set_addidtional_register(frequency_band_t band, uint8_t which_ad9361) = 0;
    virtual void set_filter_bank(double freq, std::string direction) = 0;
    virtual void set_filter_bank(boost::shared_ptr<filter_bank> filter)=0; // PH
    virtual unsigned int get_switch()=0;
    virtual void set_switch(std::uint32_t sw )=0;
    virtual void set_mb(main_board_t)=0;
    virtual void set_db(daughter_t)=0;
    virtual void set_interface(pax_iface::sptr)=0;

};

class ad9361_io
{
public:
    typedef boost::shared_ptr<ad9361_io> sptr;

    virtual ~ad9361_io() {}

    virtual boost::uint8_t peek8(boost::uint32_t reg) = 0;
    virtual void poke8(boost::uint32_t reg, boost::uint8_t val) = 0;
    // SBM, multiple spi
    virtual void poke8(std::vector<boost::uint32_t> &reg, std::vector<boost::uint8_t> &val) = 0;
    virtual boost::uint8_t peek32(boost::uint32_t reg) = 0;
    virtual void poke32(boost::uint32_t reg, boost::uint32_t val) = 0;
};


}}

#endif /* INCLUDED_AD9361_CLIENT_H */
