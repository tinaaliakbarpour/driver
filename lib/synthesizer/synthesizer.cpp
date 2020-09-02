//#include <synthesizer_impl.hpp>
/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <pax_iface.h>
#include "ADF4351.h"
#include "ADF4351_cfg.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

/******************************************************************************/
/************************ Types Definitions ***********************************/
/******************************************************************************/

class adf4351_io_spi : public adf4351_io
{
public:
    adf4351_io_spi(pax::spi_iface::sptr spi_iface, boost::uint32_t slave_num) :
        _spi_iface(spi_iface), _slave_num(slave_num) {}

    virtual ~adf4351_io_spi() {}


     void write(boost::uint32_t val)
    {
        boost::lock_guard<boost::mutex> lock(_mutex);
        pax::spi_config_t config;
        config.mosi_edge = pax::spi_config_t::EDGE_RISE;
        // NOTE: ADF4351 does not have miso input
        config.miso_edge = pax::spi_config_t::EDGE_FALL;	//TODO (Ashish): FPGA SPI workaround. This should be EDGE_RISE

//        boost::uint32_t wr_word =
//                ((boost::uint32_t(reg) << ADF4351_SPI_ADDR_SHIFT) & ADF4351_SPI_ADDR_MASK) |
//                ((boost::uint32_t(val) << ADF4351_SPI_DATA_SHIFT) & ADF4351_SPI_DATA_MASK);
        _spi_iface->write_spi(_slave_num, config, val, ADF4351_SPI_NUM_BITS);
    }

private:
    pax::spi_iface::sptr	 _spi_iface;
    boost::uint32_t	 _slave_num;
    boost::mutex		_mutex;

    //static const boost::uint32_t ADF4351_SPI_WRITE_CMD  = 0x00800000;
    //static const boost::uint32_t ADF4351_SPI_READ_CMD   = 0x00000000;
    static const boost::uint32_t ADF4351_SPI_ADDR_MASK  = 0x00000007;
    static const boost::uint32_t ADF4351_SPI_ADDR_SHIFT = 0;
    static const boost::uint32_t ADF4351_SPI_DATA_MASK  = 0xFFFFFFF8;
    static const boost::uint32_t ADF4351_SPI_DATA_SHIFT = 3;
    static const boost::uint32_t ADF4351_SPI_NUM_BITS   = 32;
};

class adf4351_ctrl_impl: public adf4351_ctrl
{
    adf4351_params::sptr _client_params;
    adf4351_io::sptr _io_iface;

public:
    typedef boost::shared_ptr<adf4351_ctrl> sptr;

    adf4351_ctrl_impl(adf4351_params::sptr  client, adf4351_io::sptr io_iface) :
        _client_params(client), _io_iface(io_iface) {
    }
    virtual ~adf4351_ctrl_impl(void) {}

private:
    struct adf4351_state
    {
            struct adf4351_platform_data	*pdata;
            uint32_t	clkin;
            uint32_t	chspc;	/* Channel Spacing */
            uint32_t	fpfd;	/* Phase Frequency Detector */
            uint32_t	min_out_freq;
            uint32_t	r0_fract;
            uint32_t	r0_int;
            uint32_t	r1_mod;
            uint32_t	r4_rf_div_sel;
            uint32_t	regs[6];
            uint32_t	regs_hw[6];
            uint32_t 	val;
    }adf4351_st;



};

adf4351_io::sptr adf4351_io::make(pax::spi_iface::sptr iface, uint32_t slave_num)
{
    return sptr(new adf4351_io_spi(iface,slave_num) );
}

