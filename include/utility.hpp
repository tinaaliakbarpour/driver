#ifndef UTILITY_HPP
#define UTILITY_HPP
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <device_addr.hpp>
#include <udp_simple.hpp>
#include <device_addr.hpp>
#include <if_addrs.hpp>
#include <fw_common.h>

PAX_API pax::device_addrs_t usrp2_find(const pax::device_addr_t  &hint_);
PAX_API pax::device_addrs_t usrp2_find();
#endif // UTILITY_HPP

