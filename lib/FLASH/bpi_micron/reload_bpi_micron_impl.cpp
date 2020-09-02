#include "reload_bpi_micron_impl.hpp"

pax::reload_bpi_micron_impl::reload_bpi_micron_impl(pax_iface::sptr _iface)
{
    iface = _iface;
    flash=pax::micron::make(iface);
}

pax::reload_bpi_micron_impl::~reload_bpi_micron_impl(){
}


boost::uint32_t pax::reload_bpi_micron_impl::calc_block_number(boost::uint32_t fpga_file_addr){

    if(fpga_file_addr >= ((flash_size_B)/(get_image_file_size())) ){
        std::cerr<< "the addr should be in range of 0 to " << ((flash_size_B)/(get_image_file_size())) <<std::endl;
        throw pax::runtime_error("wrong addr set");
    }
    return fpga_file_addr*(((get_image_file_size())/(block_size_word*2)) + 1);
//    return fpga_file_addr<<7;
}
boost::uint32_t pax::reload_bpi_micron_impl::calc_word_addr(boost::uint32_t fpga_file_addr){
    return (calc_block_number(fpga_file_addr)*(block_size_word));
}

void pax::reload_bpi_micron_impl::burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path,flash::COMMAND cmd){
    std::vector<uint16_t> vec_test= make_standard_file_16x(fpga_path,cmd);
    boost::uint32_t block_addr = calc_block_number(addr);
    boost::uint32_t fpga_image_size_byte = vec_test.size()*2;
    std::vector<boost::uint16_t> data(64*1024);
    boost::uint32_t offset=0;
    for(boost::uint32_t i=0;i<(fpga_image_size_byte);i+=(128*1024))
    {
        uint32_t   size=std::min((uint32_t)((fpga_image_size_byte/2)-offset),(uint32_t)(64*1024));
        for(uint32_t i=0;i<(64*1024);i++){
            if(i<size)
                data[i] = vec_test[i + offset];
            else
                data[i] = 0xff;
        }
        flash->micron_write(block_addr+(offset/(64*1024)),data);
        offset += size;
        std::cout<<std::dec<<"%"<<(int)((double)200*offset/(double)fpga_image_size_byte)<<std::endl;
    }
}



