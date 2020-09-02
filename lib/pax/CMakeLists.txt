
########################################################################
# Setup defines for process scheduling
########################################################################
MESSAGE(STATUS "")
MESSAGE(STATUS "Configuring priority scheduling...")
INCLUDE(CheckCXXSourceCompiles)

CHECK_CXX_SOURCE_COMPILES("
    #include <pthread.h>
    int main(){
        struct sched_param sp;
        pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
        return 0;
    }
    " HAVE_PTHREAD_SETSCHEDPARAM
)

IF(CYGWIN)
    #SCHED_RR non-operational on cygwin
    SET(HAVE_PTHREAD_SETSCHEDPARAM False)
ENDIF(CYGWIN)

CHECK_CXX_SOURCE_COMPILES("
    #include <windows.h>
    int main(){
        SetThreadPriority(GetCurrentThread(), 0);
        SetPriorityClass(GetCurrentProcess(), 0);
        return 0;
    }
    " HAVE_WIN_SETTHREADPRIORITY
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../filter_bank )

IF(HAVE_PTHREAD_SETSCHEDPARAM)
    MESSAGE(STATUS "  Priority scheduling supported through pthread_setschedparam.")
    SET(THREAD_PRIO_DEFS HAVE_PTHREAD_SETSCHEDPARAM)
    LIBPAX_APPEND_LIBS(pthread)
ELSEIF(HAVE_WIN_SETTHREADPRIORITY)
    MESSAGE(STATUS "  Priority scheduling supported through windows SetThreadPriority.")
    SET(THREAD_PRIO_DEFS HAVE_WIN_SETTHREADPRIORITY)
ELSE()
    MESSAGE(STATUS "  Priority scheduling not supported.")
    SET(THREAD_PRIO_DEFS HAVE_THREAD_PRIO_DUMMY)
ENDIF()

SET_SOURCE_FILES_PROPERTIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/thread_priority.cpp
    PROPERTIES COMPILE_DEFINITIONS "${THREAD_PRIO_DEFS}"
)



LIBPAX_REGISTER_COMPONENT("PAX" ENABLE_B200 ON "ENABLE_LIBPAX" OFF)

    LIBPAX_APPEND_SOURCES(
	${CMAKE_CURRENT_SOURCE_DIR}/buffer_pool.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/byteswap.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/exception.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/platform.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ranges.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/rx_dsp_core_200.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/rx_frontend_core_200.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/serial.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/static.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/stream.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tasks.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tx_dsp_core_200.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/udp_simple.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/udp_zero_copy.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/usrp2_fifo_ctrl.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vrt_if_packet.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/wb_iface.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/pax_iface.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/pax_impl.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/log.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/thread.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tx_dsp_core_200.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/thread_priority.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/time64_core_200.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/fpga_device.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp

    )
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../filter_bank)

if(WIN32)
    LIBPAX_APPEND_SOURCES(${CMAKE_CURRENT_SOURCE_DIR}/udp_wsa_zero_copy.cpp)
    LIBPAX_APPEND_SOURCES(${CMAKE_CURRENT_SOURCE_DIR}/udp_boost_asio_link.cpp)
else()
    LIBPAX_APPEND_SOURCES(${CMAKE_CURRENT_SOURCE_DIR}/udp_zero_copy.cpp)
    LIBPAX_APPEND_SOURCES(${CMAKE_CURRENT_SOURCE_DIR}/udp_boost_asio_link.cpp)
endif()

if(WIN32)
    LIBPAX_APPEND_LIBS(ws2_32)
endif()
