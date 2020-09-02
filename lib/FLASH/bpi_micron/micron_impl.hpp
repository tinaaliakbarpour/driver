#include <micron.hpp>
#include <byteswap.hpp>
namespace pax {

class micron_impl:public micron
{
    pax_iface::sptr iface;
public:
    micron_impl(pax_iface::sptr iface);
    bool micron_erase(boost::uint32_t block_num);
    bool micron_write(boost::uint32_t block_num,std::vector<boost::uint16_t> &data);
    boost::uint16_t micron_read(boost::uint32_t addr);
    boost::uint16_t micron_read_array(boost::uint32_t addr);

private:
    bool micron_block_unlock(uint32_t block_addr);
    bool micron_blank_check(boost::uint32_t block_addr);

};

}
