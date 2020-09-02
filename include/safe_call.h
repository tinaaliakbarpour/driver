//
// Copyright 2011 Ettus Research LLC
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

#ifndef INCLUDED_PAX_UTILS_SAFE_CALL_HPP
#define INCLUDED_PAX_UTILS_SAFE_CALL_HPP

#include <config.h>
#include <exception.hpp>
//#include <PAX/utils/log.hpp>

//! helper macro for safe call to produce warnings
#define _PAX_SAFE_CALL_WARNING(code, what) std::cout<< \
    PAX_THROW_SITE_INFO("Exception caught in safe-call.") + #code + " -> " + what \
;

/*!
 * A safe-call catches all exceptions thrown by code,
 * and creates a verbose warning about the exception.
 * Usage: PAX_SAFE_CALL(some_code_to_call();)
 * \param code the block of code to call safely
 */
#define PAX_SAFE_CALL(code) \
    try{code} \
    catch(const std::exception &e){ \
        _PAX_SAFE_CALL_WARNING(code, e.what()); \
    } \
    catch(...){ \
        _PAX_SAFE_CALL_WARNING(code, "unknown exception"); \
    }

#endif /* INCLUDED_PAX_UTILS_SAFE_CALL_HPP */
