#ifndef __servo_impl__
#define __servo_impl__


#include <pax_iface.h>
#include <servo.hpp>
#include <boost/thread.hpp>
#include <pax_regs.hpp>

namespace pax {
    class servo_impl:public servo{
      public:
        servo_impl(pax_iface::sptr iface,boost::uint16_t addr):iface(iface),addr(addr)
        {
		iface->poke32(UART_BASE+(addr*8*4),434);
        }
        ~servo_impl()
        {
            stop();
            rotate_pos(0.0,10);
        }
         void rotate_pos(double position,double speed);
         void rotate_cw(double speed);
         void rotate_ccw(double speed);
         void rotate_speed_sector(double position,double delta_pos,double speed);
         double  get_shaft_angel();
         bool run();
         bool stop();
    private:
       pax_iface::sptr iface;
       boost::uint16_t addr;
       bool running;
       void read_data();
       double angle;
       bool valid;
       boost::thread runner;
    };
}



#endif
