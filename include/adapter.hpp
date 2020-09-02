//
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_PAXLIB_TRANSPORT_ADAPTER_HPP
#define INCLUDED_PAXLIB_TRANSPORT_ADAPTER_HPP

#include <adapter_info.hpp>
#include <udp_boost_asio_link.hpp>

namespace pax { namespace transport {

class adapter_ctx : pax::noncopyable
{
public:
    PAX_SINGLETON_FCN(adapter_ctx, get);

    ~adapter_ctx() = default;

    adapter_id_t register_adapter(adapter_info& info);

private:
    adapter_ctx() = default;

    std::mutex _mutex;
    std::unordered_map<std::string, adapter_id_t> _id_map;
};

}} // namespace pax::transport

#endif /* INCLUDED_PAXLIB_TRANSPORT_ADAPTER_HPP */
