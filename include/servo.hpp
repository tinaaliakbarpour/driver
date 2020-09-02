#ifndef __servo__
#define __servo__
#include <config.h>
#include <boost/shared_ptr.hpp>

namespace pax{
    class PAX_API servo{
    public:
        typedef boost::shared_ptr<servo> sptr;
        static sptr make (pax_iface::sptr iface,boost::uint16_t addr);

        virtual void rotate_pos(double position,double speed=0.1)=0;
        virtual void rotate_cw(double speed=0.1)=0;
        virtual void rotate_ccw(double speed=0.1)=0;
        virtual void rotate_speed_sector(double position,double delta_pos=5,double speed=0.1)=0;
        virtual double  get_shaft_angel()=0;
        virtual bool run()=0;
        virtual bool stop()=0;


    };
}


#endif
