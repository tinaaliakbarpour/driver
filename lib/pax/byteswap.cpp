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
/***********************************************************************
 * Platform-specific implementation details for byteswap below:
 **********************************************************************/
#include <byteswap.hpp>
#if defined(BOOST_MSVC) //http://msdn.microsoft.com/en-us/library/a3140177%28VS.80%29.aspx
    #include <cstdlib>

    PAX_INLINE boost::uint16_t pax::byteswap(boost::uint16_t x){
        return _byteswap_ushort(x);
    }

    PAX_INLINE boost::uint32_t pax::byteswap(boost::uint32_t x){
        return _byteswap_ulong(x);
    }

    PAX_INLINE boost::uint64_t pax::byteswap(boost::uint64_t x){
        return _byteswap_uint64(x);
    }

#elif defined(__GNUC__) && __GNUC__ >= 5

    PAX_API boost::uint16_t pax::byteswap(boost::uint16_t x){
        return (x>>8) | (x<<8); //DNE return __builtin_bswap16(x);
    }

    PAX_API  boost::uint32_t pax::byteswap(boost::uint32_t x){
        return __builtin_bswap32(x);
    }

    PAX_API boost::uint64_t pax::byteswap(boost::uint64_t x){
        return __builtin_bswap64(x);
    }

#elif defined(PAX_PLATFORM_MACOS)
    #include <libkern/OSByteOrder.h>

    PAX_INLINE boost::uint16_t PAX::byteswap(boost::uint16_t x){
        return OSSwapInt16(x);
    }

    PAX_INLINE boost::uint32_t PAX::byteswap(boost::uint32_t x){
        return OSSwapInt32(x);
    }

    PAX_INLINE boost::uint64_t PAX::byteswap(boost::uint64_t x){
        return OSSwapInt64(x);
    }

#elif defined(PAX_PLATFORM_LINUX)
    #include <byteswap.h>

    PAX_INLINE boost::uint16_t PAX::byteswap(boost::uint16_t x){
        return bswap_16(x);
    }

    PAX_INLINE boost::uint32_t PAX::byteswap(boost::uint32_t x){
        return bswap_32(x);
    }

    PAX_INLINE boost::uint64_t PAX::byteswap(boost::uint64_t x){
        return bswap_64(x);
    }

#else //http://www.koders.com/c/fidB93B34CD44F0ECF724F1A4EAE3854BA2FE692F59.aspx

    PAX_INLINE boost::uint16_t PAX::byteswap(boost::uint16_t x){
        return (x>>8) | (x<<8);
    }

    PAX_INLINE boost::uint32_t PAX::byteswap(boost::uint32_t x){
        return (boost::uint32_t(PAX::byteswap(boost::uint16_t(x&0xfffful)))<<16) | (PAX::byteswap(boost::uint16_t(x>>16)));
    }

    PAX_INLINE boost::uint64_t PAX::byteswap(boost::uint64_t x){
        return (boost::uint64_t(PAX::byteswap(boost::uint32_t(x&0xffffffffull)))<<32) | (PAX::byteswap(boost::uint32_t(x>>32)));
    }

#endif

