#include <utility.hpp>
#include <pax_regs.hpp>
#include <pax_reload.hpp>
#include "recorder.hpp"
#include <eeprom.hpp>
#include <boost/program_options.hpp>


int main(int argc, char* argv[])
{

    //Establish user options
    uint32_t image_addr=(0<<31);
    std::string filepath;

    boost::program_options::options_description desc("Allowed options:");
    desc.add_options()
            ("help", "Display this help message.")
            ("addr", boost::program_options::value<uint32_t>(&image_addr), "Specify an address of flash in range 0 to 7 for write.")
            ("fpga", boost::program_options::value<std::string>(&filepath), "Specify a filepath for a custom FPGA image.")
            ("no-fpga", "Do not burn an FPGA image.")
            ("overwrite-safe", "Overwrite safe images (not recommended).")
            ("dont-check-rev", "Don't verify images are for correct model before burning.")
            ("auto-reboot", "Automatically reboot PAX without prompting.")
            ("list", "List available PAX devices.")
            ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if(vm.empty()){
        std::cout<<"use --help for help"<<std::endl;
        return EXIT_SUCCESS;
    }

    //Print help message
    if(vm.count("help") > 0){
        std::cout << boost::format("PAX Net Burner\n");
        std::cout << boost::format("Automatically burns standard FPGA images onto PAX devices.\n");
        std::cout << boost::format("Can optionally take user input for custom images.\n\n");
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    if(vm.count("addr") == 0){
        image_addr = 0;
    }


    mb_container_type tester;
    pax_init(tester,0);


        pax::pax_reload::sptr reloader = pax::pax_reload::make(tester.iface);
        reloader->burn_fpga_image(image_addr,filepath);
        //Print help message
        if(vm.count("auto-reboot") > 0){
            std::cout << boost::format("Restarting FPGA ...\n");
            reloader->restart_fpga(image_addr, 0);
            return EXIT_SUCCESS;
        }



    return 0;
}








//static u_int32_t ReadId[HWICAP_EXAMPLE_BITSTREAM_LENGTH] =
//{
//    0xFFFFFFFF, /* Dummy Word */
//    0xAA995566, /* Sync Word*/
//    0x20000000, /* Type 1 NO OP */
//    0x30020001, /* Write WBSTAR cmd */
//    0x00400000, /* Addr in SPI Flash of Multiboot bitstream */
//    0x30008001, /* Write CMD */
//    0x0000000F, /* Write IPROG */40
//    0x20000000, /* Type 1 NO OP  */
//};

//static u_int32_t ReadId[HWICAP_EXAMPLE_BITSTREAM_LENGTH] =
//{
//    XHI_DUMMY_PACKET, /* Dummy Word */
//    XHI_SYNC_PACKET, /* Sync Word*/
//    XHI_NOOP_PACKET, /* Type 1 NO OP */
//    XHI_NOOP_PACKET, /* Type 1 NO OP */
//    XHI_DEVICE_ID_READ, /* Read Product ID Code Register */
//    XHI_NOOP_PACKET, /* Type 1 NO OP */
//};
