#include <config.h>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <device_addr.hpp>
#include <udp_simple.hpp>
#include <device_addr.hpp>
#include <if_addrs.hpp>
#include <log.hpp>

using boost::asio::ip::udp;
using namespace  pax;

#include <fw_common.h>
#include <byteswap.hpp>
#include <paxtype.h>
#include <utility.hpp>
static boost::asio::ip::address_v4 sockaddr_to_ip_addr(sockaddr *addr){
    return boost::asio::ip::address_v4(ntohl(
        reinterpret_cast<sockaddr_in*>(addr)->sin_addr.s_addr
    ));
}

#ifdef PAX_PLATFORM_LINUX
#include <ifaddrs.h>



std::vector<pax::transport::if_addrs_t> pax::transport::get_if_addrs(void) {
	std::vector<if_addrs_t> if_addrs;
	struct ifaddrs *ifap;
	if (getifaddrs(&ifap) == 0) {
		for (struct ifaddrs *iter = ifap; iter != NULL; iter = iter->ifa_next) {
			//ensure that the entries are valid
			if (iter->ifa_addr == NULL) continue;
			if (iter->ifa_addr->sa_family != AF_INET) continue;
			if (iter->ifa_netmask->sa_family != AF_INET) continue;
			if (iter->ifa_broadaddr->sa_family != AF_INET) continue;

			//append a new set of interface addresses
			if_addrs_t if_addr;
			if_addr.inet = sockaddr_to_ip_addr(iter->ifa_addr).to_string();
			if_addr.mask = sockaddr_to_ip_addr(iter->ifa_netmask).to_string();
			if_addr.bcast = sockaddr_to_ip_addr(iter->ifa_broadaddr).to_string();

			//correct the bcast address when its same as the gateway
			if (if_addr.inet == if_addr.bcast or sockaddr_to_ip_addr(iter->ifa_broadaddr) == boost::asio::ip::address_v4(0)) {
				//manually calculate broadcast address
				//https://svn.boost.org/trac/boost/ticket/5198
				const boost::uint32_t addr = sockaddr_to_ip_addr(iter->ifa_addr).to_ulong();
				const boost::uint32_t mask = sockaddr_to_ip_addr(iter->ifa_netmask).to_ulong();
				const boost::uint32_t bcast = (addr & mask) | ~mask;
				if_addr.bcast = boost::asio::ip::address_v4(bcast).to_string();
			}

			if_addrs.push_back(if_addr);
		}
		freeifaddrs(ifap);
	}
	return if_addrs;
}

#endif /* HAVE_GETIFADDRS */

/***********************************************************************
* Interface address discovery through windows api
**********************************************************************/
#ifdef PAX_PLATFORM_WIN32
#include <winsock2.h>

std::vector<pax::transport::if_addrs_t> pax::transport::get_if_addrs(void) {
	std::vector<if_addrs_t> if_addrs;
	SOCKET sd = WSASocket(AF_INET, SOCK_DGRAM, 0, 0, 0, 0);
	if (sd == SOCKET_ERROR) {
		std::cerr << "Failed to get a socket. Error " << WSAGetLastError() <<
			std::endl; return if_addrs;
	}

	INTERFACE_INFO InterfaceList[20];
	unsigned long nBytesReturned;
	if (WSAIoctl(sd, SIO_GET_INTERFACE_LIST, 0, 0, &InterfaceList,
		sizeof(InterfaceList), &nBytesReturned, 0, 0) == SOCKET_ERROR) {
		std::cerr << "Failed calling WSAIoctl: error " << WSAGetLastError() <<
			std::endl;
		return if_addrs;
	}

	int nNumInterfaces = nBytesReturned / sizeof(INTERFACE_INFO);
	for (int i = 0; i < nNumInterfaces; ++i) {
		boost::uint32_t iiAddress = ntohl(reinterpret_cast<sockaddr_in&>(InterfaceList[i].iiAddress).sin_addr.s_addr);
		boost::uint32_t iiNetmask = ntohl(reinterpret_cast<sockaddr_in&>(InterfaceList[i].iiNetmask).sin_addr.s_addr);
		boost::uint32_t iiBroadcastAddress = (iiAddress & iiNetmask) | ~iiNetmask;

		if_addrs_t if_addr;
		if_addr.inet = boost::asio::ip::address_v4(iiAddress).to_string();
		if_addr.mask = boost::asio::ip::address_v4(iiNetmask).to_string();
		if_addr.bcast = boost::asio::ip::address_v4(iiBroadcastAddress).to_string();
		if_addrs.push_back(if_addr);
	}

	return if_addrs;
}

#endif /* HAVE_SIO_GET_INTERFACE_LIST */





  device_addrs_t usrp2_find(const device_addr_t  &hint_){
    //handle the multi-device discovery
    device_addrs_t hints = separate_device_addr(hint_);
    if (hints.size() > 1){
        device_addrs_t found_devices;
        std::string error_msg;
        BOOST_FOREACH(const device_addr_t &hint_i, hints){
            device_addrs_t found_devices_i = usrp2_find(hint_i);
            if (found_devices_i.size() != 1) error_msg += str(boost::format(
                "Could not resolve device hint \"%s\" to a single device."
            ) % hint_i.to_string());
            else found_devices.push_back(found_devices_i[0]);
        }
        if (found_devices.empty()) return device_addrs_t();
      //  if (not error_msg.empty()) throw pax::value_error(error_msg);
        return device_addrs_t(1, combine_device_addrs(found_devices));
    }

    //initialize the hint for a single device case
 //   PAX_ASSERT_THROW(hints.size() <= 1);
    hints.resize(1); //in case it was empty
    device_addr_t hint = hints[0];
    device_addrs_t usrp2_addrs;

    //return an empty list of addresses when type is set to non-usrp2
    if (hint.has_key("type") and hint["type"] != "usrp2") return usrp2_addrs;

    //Return an empty list of addresses when a resource is specified,
    //since a resource is intended for a different, non-USB, device.
    if (hint.has_key("resource")) return usrp2_addrs;

    //if no address was specified, send a broadcast on each interface
    if (not hint.has_key("addr")){
        BOOST_FOREACH(const pax::transport::if_addrs_t &if_addrs, pax::transport::get_if_addrs()){
            //avoid the loopback device
            if (if_addrs.inet == boost::asio::ip::address_v4::loopback().to_string()) continue;

            //create a new hint with this broadcast address
            device_addr_t new_hint = hint;
            new_hint["addr"] = if_addrs.bcast;

            //call discover with the new hint and append results
            device_addrs_t new_usrp2_addrs = usrp2_find(new_hint);
            usrp2_addrs.insert(usrp2_addrs.begin(),
                new_usrp2_addrs.begin(), new_usrp2_addrs.end()
            );
        }
        return usrp2_addrs;
    }

    //Create a UDP transport to communicate:
    //Some devices will cause a throw when opened for a broadcast address.
    //We print and recover so the caller can loop through all bcast addrs.
    pax::transport::udp_simple::sptr udp_transport;
    try{
        udp_transport = pax::transport::udp_simple::make_broadcast(hint["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT));
    }
    catch(const std::exception &e){
           PAX_LOGGER_ERROR("PAX") << boost::format("Cannot open UDP transport on %s\n%s")
                                         % hint["addr"] % e.what();
        return usrp2_addrs; //dont throw, but return empty address so caller can insert
    }

    //send a hello control packet
    usrp2_ctrl_data_t ctrl_data_out = usrp2_ctrl_data_t();
    ctrl_data_out.proto_ver =htonl(USRP2_FW_COMPAT_NUM);
    ctrl_data_out.id = htonl(USRP2_CTRL_ID_WAZZUP_BRO);
    try
    {
        udp_transport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
    }
    catch(const std::exception &ex)
    {
        PAX_LOGGER_ERROR("PAX") << "USRP2 Network discovery error " << ex.what();
    }
    catch(...)
    {
        PAX_LOGGER_ERROR("PAX")<< "USRP2 Network discovery unknown error ";
    }

    //loop and recieve until the timeout
    boost::uint8_t usrp2_ctrl_data_in_mem[pax::transport::udp_simple::mtu]; //allocate max bytes for recv
    const usrp2_ctrl_data_t *ctrl_data_in = reinterpret_cast<const usrp2_ctrl_data_t *>(usrp2_ctrl_data_in_mem);
    while(true){
        size_t len = udp_transport->recv(boost::asio::buffer(usrp2_ctrl_data_in_mem));
        if (len > offsetof(usrp2_ctrl_data_t, data) and ntohl(ctrl_data_in->id) == USRP2_CTRL_ID_WAZZUP_DUDE){

            //make a boost asio ipv4 with the raw addr in host byte order
            device_addr_t new_addr;
            new_addr["type"] = "usrp2";

            new_addr["addr"] = udp_transport->get_recv_addr();

            pax::transport::udp_simple::sptr ctrl_xport = pax::transport::udp_simple::make_connected(
                new_addr["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
            );
            ctrl_xport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
            size_t len = ctrl_xport->recv(boost::asio::buffer(usrp2_ctrl_data_in_mem));
            if (len > offsetof(usrp2_ctrl_data_t, data) and ntohl(ctrl_data_in->id) == USRP2_CTRL_ID_WAZZUP_DUDE){
                //found the device, open up for communication!
            }
            else{
                //otherwise we don't find it...
                continue;
            }



            if (
                (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
                (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
            ){
                usrp2_addrs.push_back(new_addr);
            }

            //dont break here, it will exit the while loop
            //just continue on to the next loop iteration
        }
        if (len == 0) break; //timeout
    }

    return usrp2_addrs;
}

  device_addrs_t usrp2_find(){




    device_addrs_t usrp2_addrs;

        BOOST_FOREACH(const pax::transport::if_addrs_t &if_addrs, pax::transport::get_if_addrs()){

            if (if_addrs.inet == boost::asio::ip::address_v4::loopback().to_string()) continue;

            device_addr_t new_hint ;
            new_hint["addr"] = if_addrs.bcast;

            //call discover with the new hint and append results
            device_addrs_t new_usrp2_addrs = usrp2_find(new_hint);
            usrp2_addrs.insert(usrp2_addrs.begin(),
                new_usrp2_addrs.begin(), new_usrp2_addrs.end()
            );
        }
        return usrp2_addrs;
  }



  mtu_result_t determine_mtu(const std::string &addr, const mtu_result_t &user_mtu){
    pax::transport::udp_simple::sptr udp_sock = pax::transport::udp_simple::make_connected(
        addr, BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
    );

    //The FPGA offers 4K buffers, and the user may manually request this.
    //However, multiple simultaneous receives (2DSP slave + 2DSP master),
    //require that buffering to be used internally, and this is a safe setting.
    std::vector<boost::uint8_t> buffer(std::max(user_mtu.recv_mtu, user_mtu.send_mtu));
    usrp2_ctrl_data_t *ctrl_data = reinterpret_cast<usrp2_ctrl_data_t *>(&buffer.front());
    static const double echo_timeout = 0.020; //20 ms

    //test holler - check if its supported in this fw version
    ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
    ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
    ctrl_data->data.echo_args.len = htonl(sizeof(usrp2_ctrl_data_t));
    udp_sock->send(boost::asio::buffer(buffer, sizeof(usrp2_ctrl_data_t)));
    udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);
    if (ntohl(ctrl_data->id) != USRP2_CTRL_ID_HOLLER_BACK_DUDE)
        throw pax::not_implemented_error("holler protocol not implemented");

    size_t min_recv_mtu = sizeof(usrp2_ctrl_data_t), max_recv_mtu = user_mtu.recv_mtu;
    size_t min_send_mtu = sizeof(usrp2_ctrl_data_t), max_send_mtu = user_mtu.send_mtu;

    while (min_recv_mtu < max_recv_mtu){

        size_t test_mtu = (max_recv_mtu/2 + min_recv_mtu/2 + 3) & ~3;

        ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
        ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
        ctrl_data->data.echo_args.len = htonl(test_mtu);
        udp_sock->send(boost::asio::buffer(buffer, sizeof(usrp2_ctrl_data_t)));

        size_t len = udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);

        if (len >= test_mtu) min_recv_mtu = test_mtu;
        else                 max_recv_mtu = test_mtu - 4;

    }

    while (min_send_mtu < max_send_mtu){

        size_t test_mtu = (max_send_mtu/2 + min_send_mtu/2 + 3) & ~3;

        ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
        ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
        ctrl_data->data.echo_args.len = htonl(sizeof(usrp2_ctrl_data_t));
        udp_sock->send(boost::asio::buffer(buffer, test_mtu));

        size_t len = udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);
        if (len >= sizeof(usrp2_ctrl_data_t)) len = ntohl(ctrl_data->data.echo_args.len);

        if (len >= test_mtu) min_send_mtu = test_mtu;
        else                 max_send_mtu = test_mtu - 4;
    }

    mtu_result_t mtu;
    mtu.recv_mtu = min_recv_mtu;
    mtu.send_mtu = min_send_mtu;
    return mtu;
}


