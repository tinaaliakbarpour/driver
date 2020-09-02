#include "pax_reload_impl.hpp"
#include <byteswap.hpp>

boost::uint32_t pax::pax_reload_impl::bit_reverse(boost::uint32_t x)
{
    x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
    x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
    x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
    x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));
    return((x >> 16) | (x << 16));
}

boost::uint16_t pax::pax_reload_impl::bit_reverse(boost::uint16_t in){
    boost::uint32_t x = static_cast<boost::uint32_t>(in);
    boost::uint32_t y = (bit_reverse(x))>>16;
    return static_cast<boost::uint16_t>(y);
}
boost::uint8_t pax::pax_reload_impl::bit_reverse(boost::uint8_t in){
    boost::uint32_t x = static_cast<boost::uint32_t>(in);
    boost::uint32_t y = (bit_reverse(x))>>24;
    return static_cast<boost::uint8_t>(y);
}
boost::uint16_t pax::pax_reload_impl::byte_reverse(boost::uint16_t x){
    return pax::byteswap(x);
}
boost::uint32_t pax::pax_reload_impl::byte_reverse(boost::uint32_t x){
    return pax::byteswap(x);
}

pax::pax_reload_impl::size_in_byte_t pax::pax_reload_impl::read_file(){
    flash_file_byte_8.clear();
    std::ifstream * fpga_file;
    fpga_file = new std::ifstream (file_path.c_str(), std::ios_base::binary);
    if(not fpga_file->is_open())
        throw runtime_error("can not open the file");
    fpga_file->seekg(0, std::ios::end);
    boost::uint32_t fpga_image_size = fpga_file->tellg();
    flash_file_byte_8.resize(fpga_image_size);
    fpga_file->seekg(0, std::ios::beg);
    fpga_file->read( reinterpret_cast<char*>(flash_file_byte_8.data()),fpga_image_size);
    fpga_file->close();
    delete fpga_file;
    return fpga_image_size;
}
void pax::pax_reload_impl::print_percentage_func(){
    float pass=0;
    bool diff;
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    do{
        std::cout<< pass << " %" << std::endl;
        boost::this_thread::sleep(boost::posix_time::seconds(5));
        diff = (pass != iface->get_write_Percentage_pass());
        pass=iface->get_write_Percentage_pass();
    }while(diff);
    std::cout << std::defaultfloat;

}

void pax::pax_reload_impl::run_percentage_pass_thread_show(){
    print_percentage_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&pax_reload_impl::print_percentage_func,this)));
}
void pax::pax_reload_impl::stop_percentage_pass_thread_show(){
    print_percentage_thread->join();
}
void pax::pax_reload_impl::do_additional_fpga_specific_work(uint8_vector_t& data){
    if(data.size() == 0)
        std::cerr<< "data is empty!" ;
    switch(iface->get_FPGA_series()){
    case pax::XILINX_FPGA::xilinx_six_series:{
    } break;
    case pax::XILINX_FPGA::xilinx_seven_series:{
    } break;
    default:
        throw not_implemented_error("thi type of series is not implemented yet");
        break;
    }
}

void pax::pax_reload_impl::change_file_format_to_standard(uint8_vector_t& data,supported_file_t format){
    switch(format){
    case BIN_FILE:
        _2_byte_reverse(data);
        return;
        break;
    case BIT_FILE:
        if(!detach_extera_data(data))
            throw not_implemented_error("Wrong bit file cant find header");
        bit_reverse(data);
        _2_byte_reverse(data);
        break;
    case MCS_FILE:
        throw not_implemented_error("please use this command 'pax_hex2bin -w mcs_file_path' and change it to bin file then use pax_reload again");
        break;
    case FORCE_WRITE:
        break;
    case UNKOWN:
    default:
        throw not_implemented_error("pax burner implemented for only bit file please insert a bit file or binary file");
    }
}

pax::pax_reload_impl::supported_file_t pax::pax_reload_impl::detect_file_type(flash::COMMAND cmd){
    boost::uint32_t find_loc;
    sync_word_status_t status = find_sync_word(flash_file_byte_8,find_loc);
    if(cmd == flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH){
        if( file_path.find(".mcs") != std::string::npos){
            return MCS_FILE;
        } else if (file_path.find(".bit") != std::string::npos) {
            if( (status == SYNC_WORD_DETECTED) && (check_file_size(flash_file_byte_8)) )
                return BIT_FILE;
        } else if(file_path.find(".bin") != std::string::npos) {
            if(!check_file_size(flash_file_byte_8))
                return UNKOWN;
            if(status == BITWSWAP_SYNC_WORLD_DETECTED)
                return BIN_FILE;
            else if (status != SYNC_WORD_DETECTED){
                bit_reverse(flash_file_byte_8);
                _2_byte_reverse(flash_file_byte_8);
                return BIN_FILE;
            }
        }
    } else if (cmd == flash::WRITE_IT_ANY_WAY) {
        return FORCE_WRITE;
    }
    return UNKOWN;
}

template <typename T>
bool pax::pax_reload_impl::check_file_size(const std::vector<T>& data_i){
    uint32_t image_size = (data_i.size() * sizeof(T));
    uint32_t correct_size_limit = 0;
    try{
        correct_size_limit = fpga_image_sizes[iface->get_FPGA_devices()];
    }catch (...){
        throw value_error("this type of fpga is not implemented for size checking");
    }
    uint32_t max_size_limit = correct_size_limit + 30000*sizeof(T);
    if( correct_size_limit <= image_size && max_size_limit >= image_size){
        return true;
    } else {
        return false;
    }
}

template <typename T>
pax::pax_reload_impl::sync_word_status_t pax::pax_reload_impl::find_sync_word(const std::vector<T>& data ,boost::uint32_t& detect_location){
    if (data.size() <= (3005 / sizeof(T)))
        return CANT_DETECT_SYNC_WORLD;
    for(uint32_t i = 0 ; i < (3000 / sizeof(T) ) ; i++){
        sync_word_status_t status =  check_sync_word(data[i]);
        if(status != CANT_DETECT_SYNC_WORLD){
            detect_location = i;
            return status;
        }
    }
    detect_location = 0 ;
    return CANT_DETECT_SYNC_WORLD;
}



template <typename T>
pax::pax_reload_impl::sync_word_status_t pax::pax_reload_impl::check_sync_word(T data){
    if(sizeof(T) == 1){
        return equal_to_sync_word(data);
    }
    else if (sizeof(T) == 2){
        uint8_t temp = (data & 0xff);
        sync_word_status_t status = equal_to_sync_word(temp);
        if(status != CANT_DETECT_SYNC_WORLD)
            return status;
        temp = (data >> 8);
        return equal_to_sync_word(temp);
    } else {
        throw not_implemented_error("not implemented for this size of data");
    }
}


template <typename T>
void pax::pax_reload_impl::bit_reverse(std::vector<T>& data){
    for(uint32_t i=0;i<data.size();i++)
        data[i] = bit_reverse(data[i]);
}

template <typename T>
void pax::pax_reload_impl::byte_reverse(std::vector<T>& data){
    for(uint32_t i=0;i<data.size();i++)
        data[i] = byte_reverse(data[i]);
}
void pax::pax_reload_impl::_2_byte_reverse(uint8_vector_t& x){
    for(uint32_t i=0 ; i<x.size() ; i+=2){
        uint8_t temp = x[i];
        x[i] = x[i+1];
        x[i+1] = temp;
    }
}

pax::pax_reload_impl::sync_word_status_t pax::pax_reload_impl::equal_to_sync_word(uint8_t data){
    static uint8_t sample_0=0,sample_1=0,sample_2=0,sample_3=0;
    sample_3=sample_2;
    sample_2=sample_1;
    sample_1=sample_0;
    sample_0=data;

    if( sample_0 == 0xAA && sample_1 == 0x99 && sample_2 == 0x55 && sample_3 == 0x66){
        return REVERSE_SYNC_WORLD_DETECTED;
    } else if (sample_3 == 0xAA && sample_2 == 0x99 && sample_1 == 0x55 && sample_0 == 0x66) {
        return SYNC_WORD_DETECTED;
    }
    else if( sample_0 == 0x55 && sample_1 == 0x99 && sample_2 == 0xAA && sample_3 == 0x66){
        return  BITWSWAP_REVERSE_SYNC_WORLD_DETECTED;
    } else if (sample_3 == 0x55 && sample_2 == 0x99 && sample_1 == 0xAA && sample_0 == 0x66){
        return BITWSWAP_SYNC_WORLD_DETECTED;
    }
    else {
        return CANT_DETECT_SYNC_WORLD;
    }

}

boost::uint32_t pax::pax_reload_impl::get_image_file_size(){
    try{
        return fpga_image_sizes[iface->get_FPGA_devices()];
    } catch (...){
        throw value_error("this type of fpga is not implemented for size checking");
    }
}
#include <algorithm>
bool pax::pax_reload_impl::detach_extera_data_spartan6(uint8_vector_t& data){
    std::vector<uint8_t> header;
    for(uint16_t i=0;i<16;i++)
        header.push_back(0xff);
    auto res = std::search(data.begin(), data.end(), header.begin(), header.end());
    uint32_t num_should_erase = res - data.begin();
    if(res == data.end() || num_should_erase > 5000)
        return false;
    data.erase(data.begin(),data.begin() + static_cast<boost::uint32_t>(num_should_erase));
    return true;
}

bool pax::pax_reload_impl::detach_extera_data(uint8_vector_t& data){

    if(iface->get_FPGA_family() == pax::XILINX_FPGA::xilinx_spartan)
        return detach_extera_data_spartan6(data);
    else {
        boost::int32_t num = data.size() - get_image_file_size();
        if(num < 0)
            return false;
        else if(num == 0 )
            return true;
        else {
            data.erase(data.begin(),data.begin() + static_cast<boost::uint32_t>(num));
            return true;
        }
    }


}

pax::dict<pax::XILINX_FPGA::Xilinx_Device_t,uint32_t> pax::FPGA_SIZE_STRUCT::make_sizes()
{
    pax::dict<pax::XILINX_FPGA::Xilinx_Device_t,uint32_t> FPGA_size_map;
    FPGA_size_map[pax::XILINX_FPGA::XC7K325T] = 11443612;
    FPGA_size_map[pax::XILINX_FPGA::XC7K410T] = 15877916;
    FPGA_size_map[pax::XILINX_FPGA::XC6SLX150] = 4220000;
    FPGA_size_map[pax::XILINX_FPGA::XC7V690T] = 28734812;
    return FPGA_size_map;
}

pax::uint8_vector_t& pax::pax_reload_impl::make_standard_file_8x(const std::string& addr_file_path , flash::COMMAND cmd){
    file_path = addr_file_path;
    read_file();
    supported_file_t file_format = detect_file_type(cmd);
    change_file_format_to_standard(flash_file_byte_8,file_format );
    do_additional_fpga_specific_work(flash_file_byte_8);
    return flash_file_byte_8;

}

pax::uint16_vector_t& pax::pax_reload_impl::make_standard_file_16x(const std::string& addr_file_path , flash::COMMAND cmd){
    make_standard_file_8x(addr_file_path,cmd);
    static uint16_vector_t flash_file_byte_16;
    flash_file_byte_16.resize(flash_file_byte_8.size()/2);
    for(uint32_t i = 0 ; i<flash_file_byte_16.size(); i++){
        flash_file_byte_16[i] = flash_file_byte_8[2*i];
        flash_file_byte_16[i] += (static_cast<uint16_t>(flash_file_byte_8[2*i + 1])<<8);
    }

    return flash_file_byte_16;
}

boost::uint8_t pax::pax_reload_impl::swap8(unsigned char x)
{
    unsigned char r = 0;
    r |= (x >> 7) & 0x01;
    r |= (x >> 5) & 0x02;
    r |= (x >> 3) & 0x04;
    r |= (x >> 1) & 0x08;

    r |= (x << 1) & 0x10;
    r |= (x << 3) & 0x20;
    r |= (x << 5) & 0x40;
    r |= (x << 7) & 0x80;

    return r;
}

boost::uint16_t pax::pax_reload_impl::swap16(uint16_t x)
{
    return (uint16_t)swap8(x&0xFF) | ((uint16_t)swap8((x>>8)&0xFF))<<8;
}

boost::uint32_t pax::pax_reload_impl::swap32(uint32_t x)
{
    return (uint32_t)swap16(x&0xFFFF) | ((uint32_t)swap16((x>>16)&0xFFFF))<<16;
}


void pax::pax_reload_impl::wr_icap(boost::uint32_t x,bool read_back)
{
    if(read_back) iface->poke32(0x0A000, swap32(x));//byte_reverse(x)
    else
        try{
        iface->poke32(0x0A000, swap32(x));//byte_reverse(x)
    }catch(...){
        std::cout<< "FPGA IS REASTARTING";
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));
}

boost::uint32_t pax::pax_reload_impl::rd_icap()
{
    return swap16(iface->peek32(0x0A000)); //byte_reverse(
}

////////
boost::uint16_t pax::pax_reload_impl::icap_s6_read_stat()
{
    uint16_t stat;

    //UG380 p108
    wr_icap(0xffff); //dummy word
    wr_icap(0xffff); //dummy word
    wr_icap(0xAA99); //sync word
    wr_icap(0x5566); //sync word
    wr_icap(0x2000); //NOOP
    wr_icap(0x2901); //Write Type1 packet header to read STAT register
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP
    stat = rd_icap(); //Device writes one word from the STAT register to the configuration interface
    wr_icap(0x30A1); //Type 1 Write 1 Word to CMD
    wr_icap(0x000D); //DESYNC Command
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP

    return stat;
}

void pax::pax_reload_impl::restart_spartan6(uint32_t flash_addr, uint32_t fallback_flash_addr){
    //note! t.c[0] MUST contain the byte-wide read command for the flash device used.
    //for the 25P64, and most other flash devices, this is 0x03.
    union {
        uint32_t i;
        uint16_t w[2];
        uint8_t  c[4];
    } t, s;

    t.i = flash_addr;
    t.c[0] = 0x03;//FAST_READ_CMD; 0x03
    s.i = fallback_flash_addr;
    s.c[0] = 0x03;//FAST_READ_CMD; 0x03

    //TODO: look up the watchdog timer, ensure it won't fire too soon

    //    //UG380 p126
    wr_icap(0xffff); //dummy word
    wr_icap(0xffff); //dummy word
    wr_icap(0xAA99); //sync word
    wr_icap(0x5566); //sync word
    wr_icap(0x3261); //Type 1 write General 1 (1 word)
    wr_icap(t.w[1]);//wr_icap(t.w[1]); //MultiBoot Start Address [15:0]
    wr_icap(0x3281); //Type 1 write General 2 (1 word)
    wr_icap((t.c[0]<<8)|t.c[1]); // wr_icap(0x6000);  MultiBoot Start Address [23:16] and the byte-wide read command
    wr_icap(0x32A1); //Type 1 write General 3 (1 word)
    wr_icap(s.w[1]); //Fallback Start Addres [15:0]
    wr_icap(0x32C1); //Type 1 write General 4 (1 word)
    wr_icap((s.c[0]<<8)|s.c[1]); //wr_icap(0x0300);  //Fallback Start Address [23:16] and the byte-wide read command
    wr_icap(0x30A1); //Type 1 write CMD (1 word)
    wr_icap(0x0000);
    wr_icap(0x30A1); //Type 1 write CMD (1 word)
    wr_icap(0x000E); //REBOOT command
    wr_icap(0x2000); //Type 1 NOP
    wr_icap(0x2000);
    wr_icap(0x2000);
    wr_icap(0x2000);

}

void pax::pax_reload_impl::restart_7series(boost::uint32_t flash_addr){
    wr_icap(ICAP_DUMMY_WORD);
    wr_icap(ICAP_TYPE1_NOP);
    wr_icap(ICAP_SYNC_WORD);
    wr_icap(ICAP_TYPE1_NOP);
    wr_icap(ICAP_WRITE_WBSTAR);
    wr_icap(flash_addr);
    wr_icap(ICAP_TYPE1_NOP);
    wr_icap(ICAP_WRITE_CMD);
    wr_icap(ICAP_IPROG_CMD, false);
}




void pax::pax_reload_impl::restart_fpga(boost::uint32_t image_num, boost::uint32_t fallback_flash_address){

    boost::uint32_t flash_address = calc_word_addr(image_num);
    fallback_flash_address =  calc_word_addr(fallback_flash_address);

    switch(iface->get_FPGA_series()){

    case pax::XILINX_FPGA::xilinx_six_series:
        switch (iface->get_FPGA_family()) {
        case pax::XILINX_FPGA::xilinx_spartan:
            restart_spartan6(flash_address,fallback_flash_address);
            break;
        default:
            throw pax::not_implemented_error("this family hasn't implemented");
            break;
        }
    case pax::XILINX_FPGA::xilinx_seven_series:
        restart_7series(flash_address);
        break;
    default:
        throw pax::not_implemented_error("this series hasn't implemented");
    }

}


