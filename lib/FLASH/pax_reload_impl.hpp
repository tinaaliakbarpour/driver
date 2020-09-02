#ifndef __pax_reload_impl_hpp__
#define __pax_reload_impl_hpp__

#include <dict.hpp>
#include <pax_reload.hpp>
#include <byteswap.h>
#include <pax_iface.h>

namespace pax{


struct FPGA_SIZE_STRUCT{
    static pax::dict<pax::XILINX_FPGA::Xilinx_Device_t,uint32_t> make_sizes();
    typedef pax::dict<pax::XILINX_FPGA::Xilinx_Device_t,uint32_t> fpga_conf_file_size_t;
};


class pax_reload_impl : public pax_reload {
public:
    typedef boost::shared_ptr<pax_reload> sptr;
    static sptr make(pax_iface::sptr _iface);
    virtual void burn_fpga_image(boost::uint32_t addr,const std::string& fpga_path , flash::COMMAND cmd =  flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH)=0;
    virtual void restart_fpga(boost::uint32_t image_num, boost::uint32_t fallback_flash_address = 0);

protected:
    virtual boost::uint32_t calc_word_addr(boost::uint32_t flash_addr) = 0;

    boost::uint32_t bit_reverse(boost::uint32_t x);
    boost::uint16_t bit_reverse(boost::uint16_t x);
    boost::uint8_t  bit_reverse(boost::uint8_t x);


    template <typename T>
    void bit_reverse(std::vector<T>& data);

    boost::uint16_t byte_reverse(boost::uint16_t x);
    boost::uint32_t byte_reverse(boost::uint32_t x);

    void _2_byte_reverse(uint8_vector_t& x);

    template <typename T>
    void byte_reverse(std::vector<T>& data);

    void run_percentage_pass_thread_show();
    void stop_percentage_pass_thread_show();

    uint8_vector_t& make_standard_file_8x(const std::string& addr_file_path , flash::COMMAND  cmd = flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH);
    uint16_vector_t& make_standard_file_16x(const std::string& addr_file_path , flash::COMMAND  cmd = flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH);

    boost::uint16_t icap_s6_read_stat();// changed!
    void restart_spartan6(boost::uint32_t flash_addr,boost::uint32_t fallback_flash_addr);
    void restart_7series(boost::uint32_t flash_addr);


    boost::uint32_t get_image_file_size();

private:
    typedef boost::uint32_t size_in_byte_t ;
     std::string file_path;
    typedef enum {
        BIT_FILE,
        MCS_FILE,
        BIN_FILE,
        FORCE_WRITE,
        UNKOWN
    } supported_file_t;

     typedef enum {
         SYNC_WORD_DETECTED,
         REVERSE_SYNC_WORLD_DETECTED,
         BITWSWAP_SYNC_WORLD_DETECTED,
         BITWSWAP_REVERSE_SYNC_WORLD_DETECTED,
         CANT_DETECT_SYNC_WORLD
     }sync_word_status_t;

     typedef enum {
         HEX,
         BINARY
     }read_format_t;

    size_in_byte_t read_file();

     supported_file_t detect_file_type(flash::COMMAND cmd = flash::CHECK_PROTOCOL_THEN_WRITE_TO_FLASH);
     void change_file_format_to_standard(uint8_vector_t& data,supported_file_t format);
    template <typename T>
     sync_word_status_t find_sync_word(const std::vector<T>& data , boost::uint32_t& detect_location);

     template <typename T>
     sync_word_status_t check_sync_word(T data);

     bool check_spartan_bin_file_header(uint8_t data);

     bool detach_extera_data(uint8_vector_t& data);
     bool detach_extera_data_spartan6(uint8_vector_t& data);
     sync_word_status_t equal_to_sync_word(uint8_t data);

     template<typename T>
     bool check_file_size(const std::vector<T>& data_i);


     void do_additional_fpga_specific_work(uint8_vector_t& data);

     void print_percentage_func();
     boost::shared_ptr<boost::thread> print_percentage_thread;

     void wr_icap(boost::uint32_t x,bool read_back = true);
     boost::uint32_t rd_icap();





     boost::uint8_t swap8(boost::uint8_t x);
     boost::uint16_t swap16(boost::uint16_t x);
     boost::uint32_t swap32(boost::uint32_t x);



private:
     uint8_vector_t flash_file_byte_8;
     const FPGA_SIZE_STRUCT::fpga_conf_file_size_t fpga_image_sizes =  FPGA_SIZE_STRUCT::make_sizes();



     boost::uint32_t ICAP_DUMMY_WORD           = 0xFFFFFFFF   ;// = 0xFFFFFFFF;
     boost::uint32_t ICAP_SYNC_WORD            = 0xAA995566   ;// = 0x5599AA66;
     boost::uint32_t ICAP_TYPE1_NOP            = 0x20000000   ;// = 0x04000000;
     boost::uint32_t ICAP_WRITE_WBSTAR         = 0x30020001   ;// = 0x0C400080;
     boost::uint32_t ICAP_WRITE_CMD            = 0x30008001   ;// = 0x0C000180;
     boost::uint32_t ICAP_IPROG_CMD            = 0x0000000F   ;// = 0x000000F0;




protected:
     pax_iface::sptr iface;


};


}


#endif
