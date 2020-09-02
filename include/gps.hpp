#include <pax_iface.h>

#define COMPASS_ADDR 0

namespace pax{
class PAX_API gps
{
public:
    typedef boost::shared_ptr<gps> sptr;
    static sptr make(pax_iface::sptr,boost::uint16_t addr);
    virtual float get_lat()=0;
    virtual float get_lon()=0;
    virtual float get_GMT_hour()=0;
    virtual float get_GMT_minute()=0;
    virtual float get_GMT_second()=0;

    static const boost::uint32_t UART_MAX_LEN=20;
    virtual void run()=0;
    virtual void stop()=0;

};
}
