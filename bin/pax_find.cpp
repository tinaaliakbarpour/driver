
#include <stdint.h>
#include <utility.hpp>
#include <log.hpp>
int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;
    pax::device_addr_t hint;

pax::device_addrs_t out=usrp2_find(hint);
//mtu_result_t mtu2;
//mtu_result_t user_mtu;
//user_mtu.recv_mtu = size_t(out[0].cast<double>("recv_frame_size", pax::transport::udp_simple::mtu));
//user_mtu.send_mtu = size_t(out[0].cast<double>("send_frame_size", pax::transport::udp_simple::mtu));
//mtu2=determine_mtu(out[0]["addr"],user_mtu);
BOOST_FOREACH(pax::device_addr_t& dev,out)
{
PAX_LOGGER_INFO("PAX")<<boost::format("Found PAX in %s  and OK control!\n")%dev["addr"];
}
if(out.size()==0)
{
    PAX_LOGGER_INFO("PAX")<<boost::format("NO PAX FOUND!!!");
    exit(0);
}
return 0;
}


