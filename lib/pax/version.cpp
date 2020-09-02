//
// Copyright 2010-2012 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <log.hpp>
#include <static.hpp>
#include <version.hpp>
#include <version.hpp>
#include <iostream>

std::string pax::get_version_string(void)
{
    return "2";
}

std::string pax::get_abi_string(void)
{
    return PAX_VERSION_ABI_STRING;
}

std::string pax::get_component(void)
{
    return "1";
}
