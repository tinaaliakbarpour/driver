//
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_PAXLIB_TRANSPORT_LINKS_HPP
#define INCLUDED_PAXLIB_TRANSPORT_LINKS_HPP

#include <io_service.hpp>
#include <link_if.hpp>
#include <tuple>

namespace pax { namespace transport {

enum class link_type_t { CTRL = 0, ASYNC_MSG, TX_DATA, RX_DATA };

//! Contains all information regarding a link interface
using both_links_t = std::tuple<pax::transport::send_link_if::sptr,
    size_t, // num_send_frames
    pax::transport::recv_link_if::sptr,
    size_t, // num_recv_frames
    bool, // lossy_xport
    bool>; // packet flow control

/*!
 * Parameters for link creation.
 */
struct link_params_t
{
    size_t recv_frame_size = 0;
    size_t send_frame_size = 0;
    size_t num_recv_frames = 0;
    size_t num_send_frames = 0;
    size_t recv_buff_size  = 0;
    size_t send_buff_size  = 0;
};


}} // namespace pax::transport

#endif /* INCLUDED_PAXLIB_TRANSPORT_LINKS_HPP */
