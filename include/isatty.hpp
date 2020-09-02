//
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <config.h>
#include <narrow.hpp>

namespace pax {

#ifdef PAX_PLATFORM_WIN32

#    include <io.h>

/*! Portable version of isatty()
 *
 * We call it is_a_tty() to distinguish from the from the POSIX version.
 * Also, we simply return a Boolean since the Windows version doesn't set
 * errno.
 */
bool is_a_tty(const int fd)
{
    return pax::narrow_cast<bool>(_isatty(fd));
}

#elif _POSIX_C_SOURCE >= _200112L

#    include <unistd.h>

/*! Portable version of isatty()
 *
 * We call it is_a_tty() to distinguish from the from the POSIX version.
 * Also, we simply return a Boolean since the Windows version doesn't set
 * errno.
 */
bool is_a_tty(const int fd)
{
    return isatty(fd);
}

#else

bool is_a_tty(const int fd)
{
    return false;
}

#endif

} /* namespace pax */
