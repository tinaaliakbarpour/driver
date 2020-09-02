#include "reload_BPI_S29GL01GS_impl.hpp"

pax::reload_BPI_S29GL01GS_impl::reload_BPI_S29GL01GS_impl(pax_iface::sptr _iface)
{
    iface = _iface;
}

pax::reload_BPI_S29GL01GS_impl::~reload_BPI_S29GL01GS_impl(){
}

void pax::reload_BPI_S29GL01GS_impl::burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path,flash::COMMAND cmd){
    std::vector<uint16_t> vec_final= make_standard_file_16x(fpga_path,cmd);
    std::cout<< "ERASING FLASH PLEASE WAIT"<<std::endl;
    iface->erase_flash(addr,vec_final.size());
    std::cout<< "ERASING DONE"<<std::endl;
    std::cout<< "writing data to flash it might take several minutes"<<std::endl;
    run_percentage_pass_thread_show();
    iface->write_flash(addr,vec_final);
    stop_percentage_pass_thread_show();
    std::cout<< "done!"<<std::endl;
}

boost::uint32_t  pax::reload_BPI_S29GL01GS_impl::calc_word_addr(uint32_t flash_addr){
    flash_addr = flash_addr;
    throw not_implemented_error("not implemented yet");
    return 0;
}
