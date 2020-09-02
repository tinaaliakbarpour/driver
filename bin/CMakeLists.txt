#
# Copyright 2010-2014 Ettus Research LLC
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

SET(CMAKE_C_COMPILE_OBJECT ${CMAKE_CXX_COMPILE_OBJECT})

########################################################################
# Utilities that get installed into the runtime path
########################################################################
SET(util_runtime_sources
    pax_find.cpp
    pax_burner.cpp
    pax_restart.cpp
    rx1_example.cpp
    rx_sync_example.cpp
    ip_change.cpp
    rx_rf_sync_example.cpp
    tx1_example.cpp
    uart_terminal.cpp
    rx_log_example.cpp
    link.cpp
    multi_pax.cpp
    my_test.cpp
)

SET(x3xx_burner_sources
    usrp_x3xx_fpga_burner.cpp
    cdecode.c
)

find_package(UDev)
IF(ENABLE_E300)
    IF(UDEV_FOUND)
        LIST(APPEND util_runtime_sources usrp_e3x0_network_mode.cpp)
    ENDIF(UDEV_FOUND)
ENDIF(ENABLE_E300)

#for each source: build an executable and install
FOREACH(util_source ${util_runtime_sources})
    GET_FILENAME_COMPONENT(util_name ${util_source} NAME_WE)
    ADD_EXECUTABLE(${util_name} ${util_source} recorder.cpp)
    TARGET_LINK_LIBRARIES(${util_name} pax ${Boost_LIBRARIES})
    PAX_INSTALL(TARGETS ${util_name} RUNTIME DESTINATION ${RUNTIME_DIR} COMPONENT utilities)
ENDFOREACH(util_source)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../python_lib/usrp2_recovery.py
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../python_lib/)
ADD_DEFINITIONS(-Dpython_script_path="${CMAKE_CURRENT_BINARY_DIR}/../python_lib/")
