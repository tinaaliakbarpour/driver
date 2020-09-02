#ifndef __reload_bpi_micron_impl_hpp__
#define __reload_bpi_micron_impl_hpp__

#include "../pax_reload_impl.hpp"
#include <byteswap.hpp>

namespace pax {

class reload_bpi_micron_impl:public pax_reload_impl
{
public:
    pax::pax_reload::sptr make(pax_iface::sptr iface);
    reload_bpi_micron_impl(pax_iface::sptr iface);
    ~reload_bpi_micron_impl();
    virtual void burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path , flash::COMMAND cmd = flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH);
protected:
    boost::uint32_t calc_word_addr(boost::uint32_t fpga_file_addr);
private:
    pax::micron::sptr flash;
    byte_vector_t fpga_image;
    const boost::uint32_t flash_size_B = 128*1024*1024;
    const boost::uint32_t block_size_word = 0x10000;
    boost::uint32_t calc_block_number(boost::uint32_t fpga_file_addr);
};

}

#endif
