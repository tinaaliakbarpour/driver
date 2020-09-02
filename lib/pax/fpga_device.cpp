#include <fpga_device.hpp>

using namespace pax::XILINX_FPGA;

Xilinx_fpga_series_t pax::fpga_device::get_FPGA_series(){
    return FPGA_series_;
}

Xilinx_fpga_family_t pax::fpga_device::get_FPGA_family(){
    return FPGA_family_;
}

Xilinx_Device_t pax::fpga_device::get_FPGA_devices(){
    return FPGA_device_;
}


void pax::fpga_device::_set_FPGA_devices(XILINX_FPGA::Xilinx_Device_t _dev){
    FPGA_device_ = _dev;
}
void pax::fpga_device::set_FPGA_devices(Xilinx_Device_t _dev){
    _set_FPGA_devices(_dev);
    _set_FPGA_family(_dev);
    _set_FPGA_series(_dev);
}

void pax::fpga_family::_set_FPGA_family(Xilinx_Device_t _dev){
    switch (_dev) {
    case XC6SLX4:
    case XC6SLX9:
    case XCSLLX16:
    case XC6SLX25:
    case XC6SLX45:
    case XCSLLX75:
    case XC6SLX100:
    case XC6SLX150:
    case XC6SLX25T:
    case XC6SLX45T:
    case XCSLLX75T:
    case XC6SLX100T:
    case XC6SLX150T:
    case XC7S6:
    case XC7S15:
    case XC7S25:
    case XC7S50:
    case XC7S75:
    case XC7S100:
        FPGA_family_ = xilinx_spartan; break;
    case XC7A12T:
    case XC7A15T:
    case XC7A25T:
    case XC7A35T:
    case XC7A50T:
    case XC7A75T:
    case XC7A100T:
    case XC7A200T:
        FPGA_family_ = xilinx_artix; break;
    case XC7K70T:
    case XC7K160T:
    case XC7K325T:
    case XC7K355T:
    case XC7K410T:
    case XC7K420T:
    case XC7K480T:
        FPGA_family_ = xilinx_kintex; break;
    case XC7V585T:
    case XC7V2000T:
    case XC7V330T:
    case XC7V415T:
    case XC7V485T:
    case XC7V550T:
    case XC7V690T:
    case XC7V980T:
    case XC7V1140T:
    case XC7V580T:
    case XC7V870T:
        FPGA_family_ = xilinx_virtex; break;
    default:
        throw pax::value_error("this FPGA family is not ok");
        break;
    }
}
void pax::fpga_series::_set_FPGA_series(Xilinx_Device_t _dev){
    switch (_dev) {
    case XC6SLX4:
    case XC6SLX9:
    case XCSLLX16:
    case XC6SLX25:
    case XC6SLX45:
    case XCSLLX75:
    case XC6SLX100:
    case XC6SLX150:
    case XC6SLX25T:
    case XC6SLX45T:
    case XCSLLX75T:
    case XC6SLX100T:
    case XC6SLX150T:
        FPGA_series_ = xilinx_six_series; break;
    case XC7S6:
    case XC7S15:
    case XC7S25:
    case XC7S50:
    case XC7S75:
    case XC7S100:
    case XC7A12T:
    case XC7A15T:
    case XC7A25T:
    case XC7A35T:
    case XC7A50T:
    case XC7A75T:
    case XC7A100T:
    case XC7A200T:
    case XC7K70T:
    case XC7K160T:
    case XC7K325T:
    case XC7K355T:
    case XC7K410T:
    case XC7K420T:
    case XC7K480T:
    case XC7V585T:
    case XC7V2000T:
    case XC7V330T:
    case XC7V415T:
    case XC7V485T:
    case XC7V550T:
    case XC7V690T:
    case XC7V980T:
    case XC7V1140T:
    case XC7V580T:
    case XC7V870T:
        FPGA_series_ = xilinx_seven_series;  break;
    default:
        throw pax::value_error("this FPGA series is not ok");
        break;
    }
}
