#include <utility.hpp>
#include <pax_regs.hpp>
#include <pax_reload.hpp>
#include <boost/program_options.hpp>
#include <log.hpp>

int main(int argc, char *argv[])
{
    //Establish user options
    uint32_t image_addr=(0<<31);
    std::string filepath;

    boost::program_options::options_description desc("Allowed options:");
    desc.add_options()
            ("help", "Display this help message.")
            ("addr", boost::program_options::value<uint32_t>(&image_addr), "Specify an address of flash to load from.")
            ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    //Print help message
    if(vm.count("help") > 0){
        std::cout << boost::format("PAX Restarter\n");
        std::cout << boost::format("Restarts PAX devices reloading FPGA bitstream from flash.\n\n");
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    const  pax::device_addr_t hint;
    pax::device_addrs_t out = usrp2_find(hint);
    BOOST_FOREACH(pax::device_addr_t& dev,out)
    {
        std::cout<<boost::format("Found PAX in %s  and OK control!\n")%dev["addr"];
    }
    if(out.size()==0)
    {
        PAX_LOGGER_FATAL("PAX")<<boost::format("NO PAX FOUND!!!")<<std::endl;
        exit(0);
    }

    pax_iface::sptr iface = pax_iface::make(pax::transport::udp_simple::make_connected(
                                                  out[0]["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
            ));
    PAX_LOGGER_INFO("PAX")<<"firmware version is: "<<iface->get_fw_version_string()<<std::endl;
    const boost::uint32_t fpga_compat_num =iface->peek32(U2_REG_COMPAT_NUM_RB);
    boost::uint16_t fpga_major = fpga_compat_num >> 16;

    if (fpga_major != USRP2_FPGA_COMPAT_NUM){
    }
    iface->lock_device(true);

    pax::pax_reload::sptr reloader = pax::pax_reload::make(iface);
    reloader->restart_fpga(image_addr,0);
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
