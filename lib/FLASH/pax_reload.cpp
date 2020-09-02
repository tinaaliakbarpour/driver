#include "./bpi_micron/reload_bpi_micron_impl.hpp"
#include "./SPI_flash_IS25LPxxx/reload_SPI_IS25LPxxx_impl.hpp"
#include "./BPI_flash_S29GL01GS/reload_BPI_S29GL01GS_impl.hpp"




pax::pax_reload::sptr pax::pax_reload::make(pax_iface::sptr iface)
{
    flash::flash_type_t type = iface->get_flash_type();
    switch(type){
    case flash::BPI_28F00AP30:    return pax::pax_reload::sptr(new pax::reload_bpi_micron_impl(iface));  break;
    case flash::SPI_IS25LPxxx:    return pax::pax_reload::sptr(new pax::reload_SPI_IS25LPxxx_impl(iface)); break;
    case flash::BPI_S29GL01GS:    return pax::pax_reload::sptr(new pax::reload_BPI_S29GL01GS_impl(iface)); break;
    default: throw value_error("this value is not supported");
    }
}




