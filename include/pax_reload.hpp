#include <fstream>
#include <exception.hpp>
#include <boost/cstdint.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <pax_iface.h>
#include <micron.hpp>
#include <boost/filesystem.hpp>

#define FPGA_IMAGE_SIZE_BYTES  (4220212) /*11443612*/
#define FPGA_IMAGE_OFFSET  0

namespace pax {

namespace flash {

typedef enum {
    CHECK_PROTOCOL_THEN_WRITE_TO_FLASH,
    WRITE_IT_ANY_WAY
}COMMAND;

}

class PAX_API pax_reload
{
public:
    typedef boost::shared_ptr<pax_reload> sptr;
    static sptr make(pax_iface::sptr iface);
    virtual void burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path,flash::COMMAND cmd = flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH)=0;
    virtual void restart_fpga(boost::uint32_t flash_address, uint32_t fallback_flash_address = 0)=0;

};
}



