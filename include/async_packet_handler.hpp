//
// Copyright 2012 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef INCLUDED_LIBPAX_USRP_COMMON_ASYNC_PACKET_HANDLER_HPP
#define INCLUDED_LIBPAX_USRP_COMMON_ASYNC_PACKET_HANDLER_HPP

#include <config.h>
#include <vrt_if_packet.hpp>
#include <metadata.hpp>
#include <byteswap.hpp>
#include <iostream>
#include <log.hpp>

namespace pax{ namespace pax2{

    template <typename to_host_type>
    void load_metadata_from_buff(
        const to_host_type &to_host,
        async_metadata_t &metadata,
        const transport::vrt::if_packet_info_t &if_packet_info,
        const boost::uint32_t *vrt_hdr,
        const double tick_rate,
        const size_t channel = 0
    ){
        const boost::uint32_t *payload = vrt_hdr + if_packet_info.num_header_words32;

        //load into metadata
        metadata.channel = channel;
        metadata.has_time_spec = if_packet_info.has_tsf;
        metadata.time_spec = time_spec_t::from_ticks(if_packet_info.tsf, tick_rate);
        metadata.event_code = async_metadata_t::event_code_t(to_host(payload[0]) & 0xff);

        //load user payload
        for (size_t i = 1; i < if_packet_info.num_payload_words32; i++){
            if (i-1 == 4) break; //limit of 4 words32
            metadata.user_payload[i-1] = to_host(payload[i]);
        }
    }

   PAX_INLINE void standard_async_msg_prints(const async_metadata_t &metadata)
    {

    if (metadata.event_code
        & (async_metadata_t::EVENT_CODE_UNDERFLOW
              | async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET)) {
        PAX_LOG_FASTPATH("U")
    } else if (metadata.event_code
               & (async_metadata_t::EVENT_CODE_SEQ_ERROR
                     | async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST)) {
        PAX_LOG_FASTPATH("S")
    } else if (metadata.event_code & async_metadata_t::EVENT_CODE_TIME_ERROR) {
        PAX_LOG_FASTPATH("L")
    }
    }


}} //namespace pax::usrp

#endif /* INCLUDED_LIBPAX_USRP_COMMON_ASYNC_PACKET_HANDLER_HPP */
