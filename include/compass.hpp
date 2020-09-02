#ifndef __COMPASS_HPP__
#define __COMPASS_HPP__

#include <pax_iface.h>

#define COMPASS_ADDR 0

namespace pax{
class PAX_API compass
{
public:
    typedef boost::shared_ptr<compass> sptr;
    static sptr make(pax_iface::sptr,boost::uint16_t addr);
    virtual float get_heading()=0;
    virtual void run()=0;
    virtual void stop()=0;
    static const boost::uint32_t UART_MAX_LEN=20;
};
}


#endif
