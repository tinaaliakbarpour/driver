#ifndef __spi_wb_iface_hpp_
#define __spi_wb_iface_hpp_

#include <serial.hpp>
#include <wb_iface.hpp>
#include <boost/shared_ptr.hpp>



class spi_wb_iface: public pax::wb_iface, public pax::spi_iface
{
public:
    typedef boost::shared_ptr<spi_wb_iface> sptr;


};




#endif
