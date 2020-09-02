//
// Copyright 2010-2011 Ettus Research LLC
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

#ifndef INCLUDED_PAX_UTILS_BYTESWAP_HPP
#define INCLUDED_PAX_UTILS_BYTESWAP_HPP

#include <config.h>
#include <boost/cstdint.hpp>

/*! \file byteswap.hpp
 * Provide fast byteswaping routines for 16, 32, and 64 bit integers,
 * by using the system's native routines/intrinsics when available.
 */
#include <boost/detail/endian.hpp>
namespace pax{

    //! perform a byteswap on a 16 bit integer
   PAX_API boost::uint16_t byteswap(boost::uint16_t);

    //! perform a byteswap on a 32 bit integer
   PAX_API boost::uint32_t byteswap(boost::uint32_t);

    //! perform a byteswap on a 64 bit integer
   PAX_API boost::uint64_t byteswap(boost::uint64_t);



	template<typename T>  T ntohx(T num) {
#ifdef BOOST_BIG_ENDIAN
		return num;
#else
		return pax::byteswap(num);
#endif
	}

	template<typename T>  T htonx(T num) {
#ifdef BOOST_BIG_ENDIAN
		return num;
#else
		return pax::byteswap(num);
#endif
	}

	template<typename T>  T wtohx(T num) {
#ifdef BOOST_BIG_ENDIAN
		return pax::byteswap(num);
#else
		return num;
#endif
	}

	template<typename T>  T htowx(T num) {
#ifdef BOOST_BIG_ENDIAN
		return pax::byteswap(num);
#else
		return num;
#endif
	}


} //namespace PAX

#include <byteswap.ipp>

#endif /* INCLUDED_PAX_UTILS_BYTESWAP_HPP */
