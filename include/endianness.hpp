//
// Copyright 2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#ifndef INCLUDED_PAX_TYPES_ENDIANNESS_HPP
#define INCLUDED_PAX_TYPES_ENDIANNESS_HPP

#include <config.h>

/******************************************************************************
 * Detect host endianness
 *****************************************************************************/
#include <boost/predef/other/endian.h>

// In Boost 1.55, the meaning of the macros changed. They are now always
// defined, but don't always have the same value.
#if BOOST_ENDIAN_BIG_BYTE
#    define PAX_BIG_ENDIAN
#elif BOOST_ENDIAN_LITTLE_BYTE
#    define PAX_LITTLE_ENDIAN
#else
#    error "Unsupported endianness!"
#endif


namespace pax {

enum endianness_t { ENDIANNESS_BIG, ENDIANNESS_LITTLE };

} // namespace pax

#endif /* INCLUDED_PAX_TYPES_ENDIANNESS_HPP */
