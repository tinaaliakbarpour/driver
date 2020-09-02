//
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_PAX_TRANSPORT_ADAPTER_ID_HPP
#define INCLUDED_PAX_TRANSPORT_ADAPTER_ID_HPP

#include <cstddef>

namespace pax { namespace transport {

//! Host transport adapter ID
using adapter_id_t = size_t;
//! NULL/unassigned host transport adapter ID
static const adapter_id_t NULL_ADAPTER_ID = 0;

}} // namespace pax::transport

#endif /* INCLUDED_PAX_TRANSPORT_ADAPTER_ID_HPP */
