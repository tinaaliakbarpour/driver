#include <cmath>
#include <exception.hpp>
#include <boost/cstdint.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/scoped_array.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions.hpp>
#include <pax_iface.h>



#ifndef INCLUDED_MICRON_HPP
#define INCLUDED_MICRON_HPP

namespace pax {
class PAX_API micron
{
public:
    typedef boost::shared_ptr<micron> sptr;
    static sptr make( pax_iface::sptr iface);
    virtual bool micron_erase(boost::uint32_t block_num)=0;
    virtual bool micron_write(boost::uint32_t block_num,std::vector<boost::uint16_t> &data)=0;
    virtual boost::uint16_t micron_read(boost::uint32_t addr)=0;
    virtual  boost::uint16_t micron_read_array(boost::uint32_t addr) = 0;

};
}

#endif
