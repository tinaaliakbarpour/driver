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

#ifndef INCLUDED_PAX_UTILS_ASSERT_HAS_IPP
#define INCLUDED_PAX_UTILS_ASSERT_HAS_IPP

#include <algorithm.hpp>
#include <exception.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace pax{

    template<typename T, typename Range> PAX_INLINE void assert_has(
        const Range &range,
        const T &value,
        const std::string &what
    ){
        if (pax::has(range, value)) return;
        std::string possible_values = "";
        size_t i = 0;
        BOOST_FOREACH(const T &v, range){
            if (i++ > 0) possible_values += ", ";
            possible_values += boost::lexical_cast<std::string>(v);
        }
        throw pax::assertion_error(str(boost::format(
                "assertion failed:\n"
                "  %s is not a valid %s.\n"
                "  possible values are: [%s].\n"
            )
            % boost::lexical_cast<std::string>(value)
            % what % possible_values
        ));
    }

}//namespace pax

#endif /* INCLUDED_PAX_UTILS_ASSERT_HAS_IPP */
