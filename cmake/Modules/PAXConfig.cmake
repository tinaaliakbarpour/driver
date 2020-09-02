#
# Copyright 2014 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
########################################################################
#
# Find the header <pax/config.hpp> and library "libpax" for the USRP
# Hardware Driver.  Priorty for prefix search is:
# 1) ENV(PAX_DIR)
# 2) pkg-config results, if available;
# 3) CMAKE_INSTALL_PREFIX
# 4) /usr/local/
# 5) /usr/
#
# Version info is handled by PAXConfigVersion.cmake only; not here.
#
########################################################################

# set that this file was found, for use in GNU Radio's FindPAX.cmake.
# Have to use the ENV, since this file might not allow CACHE changes.

set(ENV{PAX_CONFIG_USED} TRUE)

# set default values

SET(PAX_FOUND TRUE)
SET(PAX_INCLUDE_HINTS)
SET(PAX_LIBDIR_HINTS)
SET(PAX_DIR $ENV{PAX_DIR})

IF(PAX_DIR)
    LIST(APPEND PAX_INCLUDE_HINTS ${PAX_DIR}/include)
    LIST(APPEND PAX_LIBDIR_HINTS ${PAX_DIR}/lib)
ENDIF()

INCLUDE(FindPkgConfig)
IF(PKG_CONFIG_FOUND)
  IF(NOT ${CMAKE_VERSION} VERSION_LESS "2.8.0")
    SET(PAX_QUIET "QUIET")
  ENDIF()
  PKG_CHECK_MODULES(PC_PAX ${PAX_QUIET} pax)
  IF(PC_PAX_FOUND)
    LIST(APPEND PAX_INCLUDE_HINTS ${PC_PAX_INCLUDEDIR})
    LIST(APPEND PAX_LIBDIR_HINTS ${PC_PAX_LIBDIR})
  ENDIF()
ENDIF()

LIST(APPEND PAX_INCLUDE_HINTS ${CMAKE_INSTALL_PREFIX}/include)
LIST(APPEND PAX_LIBDIR_HINTS ${CMAKE_INSTALL_PREFIX}/lib)

# Verify that <pax/config.hpp> and libpax are available, and, if a
# version is provided, that PAX meets the version requirements -- no
# matter what pkg-config might think.

FIND_PATH(
    PAX_INCLUDE_DIRS
    NAMES pax/config.hpp
    HINTS ${PAX_INCLUDE_HINTS}
    PATHS /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    PAX_LIBRARIES
    NAMES pax
    HINTS ${PAX_LIBDIR_HINTS}
    PATHS /usr/local/lib
          /usr/lib
)

IF(PAX_LIBRARIES AND PAX_INCLUDE_DIRS)

  INCLUDE(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(PAX DEFAULT_MSG PAX_LIBRARIES PAX_INCLUDE_DIRS)
  MARK_AS_ADVANCED(PAX_LIBRARIES PAX_INCLUDE_DIRS)

ELSEIF(PAX_FIND_REQUIRED)

  MESSAGE(FATAL_ERROR "PAX is required, but was not found.")

ENDIF()
