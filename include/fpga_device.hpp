#ifndef __FPGA_TYPE_HPP__
#define __FPGA_TYPE_HPP__

#include <string>
#include <boost/cstdint.hpp>
#include <exception.hpp>
namespace pax{

namespace XILINX_FPGA {

typedef enum {
    xilinx_seven_series,
    xilinx_six_series
} Xilinx_fpga_series_t;
typedef enum {
    xilinx_spartan,
    xilinx_artix,
    xilinx_kintex,
    xilinx_virtex
} Xilinx_fpga_family_t;
typedef enum {
    XC6SLX4,
    XC6SLX9,
    XCSLLX16,
    XC6SLX25,
    XC6SLX45,
    XCSLLX75,
    XC6SLX100,
    XC6SLX150,
    XC6SLX25T,
    XC6SLX45T,
    XCSLLX75T,
    XC6SLX100T,
    XC6SLX150T,
    XC7S6,
    XC7S15,
    XC7S25,
    XC7S50,
    XC7S75,
    XC7S100,
    XC7A12T,
    XC7A15T,
    XC7A25T,
    XC7A35T,
    XC7A50T,
    XC7A75T,
    XC7A100T,
    XC7A200T,
    XC7K70T,
    XC7K160T,
    XC7K325T,
    XC7K355T,
    XC7K410T,
    XC7K420T,
    XC7K480T,
    XC7V585T,
    XC7V2000T,
    XC7V330T,
    XC7V415T,
    XC7V485T,
    XC7V550T,
    XC7V690T,
    XC7V980T,
    XC7V1140T,
    XC7V580T,
    XC7V870T,
    UNKOWN
} Xilinx_Device_t;

}

class fpga_family{
public:
    virtual XILINX_FPGA::Xilinx_fpga_family_t get_FPGA_family() = 0;
protected:
    XILINX_FPGA::Xilinx_fpga_family_t FPGA_family_;
    void _set_FPGA_family(XILINX_FPGA::Xilinx_Device_t _dev);
};

class fpga_series : public fpga_family{
public:
    virtual XILINX_FPGA::Xilinx_fpga_series_t get_FPGA_series() = 0;
protected:
    XILINX_FPGA::Xilinx_fpga_series_t FPGA_series_;
    void _set_FPGA_series(XILINX_FPGA::Xilinx_Device_t _dev);
};

class fpga_device : public fpga_series{
public:
    virtual XILINX_FPGA::Xilinx_fpga_series_t get_FPGA_series();
    virtual XILINX_FPGA::Xilinx_fpga_family_t get_FPGA_family();
    XILINX_FPGA::Xilinx_Device_t get_FPGA_devices();
    void set_FPGA_devices(XILINX_FPGA::Xilinx_Device_t _dev);
protected:
    void _set_FPGA_devices(XILINX_FPGA::Xilinx_Device_t _dev);
private:
    XILINX_FPGA::Xilinx_Device_t FPGA_device_;
};

}

#endif
