#include "reload_SPI_IS25LPxxx_impl.hpp"

pax::reload_SPI_IS25LPxxx_impl::reload_SPI_IS25LPxxx_impl(pax_iface::sptr _iface)
{
    iface = _iface;
}

pax::reload_SPI_IS25LPxxx_impl::~reload_SPI_IS25LPxxx_impl(){
}

void pax::reload_SPI_IS25LPxxx_impl::burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path,flash::COMMAND cmd){
    std::vector<uint8_t> vec_temp= make_standard_file_8x(fpga_path,cmd);
    std::vector<uint16_t> vec_final;
    vec_final.reserve(vec_temp.size());
    if((vec_temp.size()%2) == 1)
        vec_temp.push_back(0xFF);
    for(uint32_t i=0;i<vec_temp.size()/2;i++){
        vec_final.push_back(bit_reverse(vec_temp[2*i+1]));
        vec_final.push_back(bit_reverse(vec_temp[2*i]));
    }
    std::cout<< "ERASING FLASH PLEASE WAIT"<<std::endl;
    iface->erase_flash(0,vec_final.size());
//    iface->erase_full_flash();
    std::cout<< "ERASING DONE"<<std::endl;
    std::cout<< "writing data to flash it might take several minutes"<<std::endl;
    run_percentage_pass_thread_show();
    iface->write_flash(addr,vec_final);
    stop_percentage_pass_thread_show();
    std::cout<< "done!"<<std::endl;
}

boost::uint32_t pax::reload_SPI_IS25LPxxx_impl::calc_word_addr(uint32_t flash_addr){
    flash_addr = flash_addr;
    throw not_implemented_error("not implemented yet");
    return 0;
}


