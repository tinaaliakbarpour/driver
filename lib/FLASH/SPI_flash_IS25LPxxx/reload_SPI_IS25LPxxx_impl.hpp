#ifndef __reload_SPI_IS25LPxxx_impl_hpp__
#define __reload_SPI_IS25LPxxx_impl_hpp__

#include "../pax_reload_impl.hpp"
#include <byteswap.hpp>

namespace pax {

class reload_SPI_IS25LPxxx_impl:public pax_reload_impl
{
public:
    pax::pax_reload::sptr make(pax_iface::sptr iface);
    reload_SPI_IS25LPxxx_impl(pax_iface::sptr iface);
    ~reload_SPI_IS25LPxxx_impl();
    virtual void burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path , flash::COMMAND cmd = flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH);

protected:
    virtual boost::uint32_t calc_word_addr(uint32_t flash_addr);

};

}

#endif
