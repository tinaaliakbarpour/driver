//
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_PAXLIB_TRANSPORT_ADAPTER_INFO_HPP
#define INCLUDED_PAXLIB_TRANSPORT_ADAPTER_INFO_HPP

#include <adapter_id.hpp>
#include <noncopyable.hpp>
#include <static.hpp>
#include <unordered_map>
#include <memory>
#include <mutex>

namespace pax { namespace transport {

class adapter_info
{
public:
    /*! Returns a unique string identifying the adapter
     *  String contents are not API. Only uniqueness is guaranteed.
     */
    virtual std::string to_string() = 0;
};

}} // namespace pax::transport

#endif /* INCLUDED_PAXLIB_TRANSPORT_ADAPTER_INFO_HPP */
