//
// Copyright 2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <config.h>
#include <boost/current_function.hpp>
#include <boost/thread/thread.hpp>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <thread.hpp>
/*! \file log.hpp
 *
 * \section loghpp_logging The PAX logging facility
 *
 * The logger enables PAX library code to easily log events into a file and
 * display messages above a certain level in the terminal.
 * Log entries are time-stamped and stored with file, line, and function.
 * Each call to the PAX_LOG macros is thread-safe. Each thread will aquire the
 * lock for the logger.
 *
 * Note: More information on the logging subsystem can be found on
 * \ref page_logging.
 *
 * To disable console logging completely at compile time specify
 * `-DPAX_LOG_CONSOLE_DISABLE` during configuration with CMake.
 *
 * By default no file logging will occur. Set a log file path:
 *  - at compile time by specifying `-DPAX_LOG_FILE=$file_path`
 *  - and/or override at runtime by setting the environment variable
 *    `PAX_LOG_FILE`
 *
 * \subsection loghpp_levels Log levels
 *
 * See also \ref logging_levels.
 *
 * All log messages with verbosity greater than or equal to the log level
 * (in other words, as often or less often than the current log level)
 * are recorded to std::clog and/or the log file.
 * Log levels can be specified using string or numeric values of
 * pax::log::severity_level.
 *
 * The default log level is "info", but can be overridden:
 *  - at compile time by setting the pre-processor define `-DPAX_LOG_MIN_LEVEL`.
 *  - at runtime by setting the environment variable `PAX_LOG_LEVEL`.
 *  - for console logging by setting `(-D)PAX_LOG_CONSOLE_LEVEL` at
 *    run-/compiletime
 *  - for file logging by setting `(-D)PAX_LOG_FILE_LEVEL` at run-/compiletime
 *
 * PAX_LOG_LEVEL can be the name of a verbosity enum or integer value:
 *   - Example pre-processor define: `-DPAX_LOG_MIN_LEVEL=3`
 *   - Example pre-processor define: `-DPAX_LOG_MIN_LEVEL=info`
 *   - Example environment variable: `export PAX_LOG_LEVEL=3`
 *   - Example environment variable: `export PAX_LOG_LEVEL=info`
 *
 * \subsection loghpp_formatting Log formatting
 *
 * The log format for messages going into a log file is CSV.
 * All log messages going into a logfile will contain following fields:
 * - timestamp
 * - thread-id
 * - source-file + line information
 * - severity level
 * - component/channel information which logged the information
 * - the actual log message
 *
 * The log format of log messages displayed on the terminal is plain text with
 * space separated tags prepended.
 * For example:
 *    - `[INFO] [X300] This is a informational log message`
 *
 * The log format for log output on the console by using these preprocessor
 * defines in CMake:
 * - `-DPAX_LOG_CONSOLE_TIME` adds a timestamp [2017-01-01 00:00:00.000000]
 * - `-DPAX_LOG_CONSOLE_THREAD` adds a thread-id `[0x001234]`
 * - `-DPAX_LOG_CONSOLE_SRC` adds a sourcefile and line tag `[src_file:line]`
 */

/*
 * Advanced logging macros
 * PAX_LOG_MIN_LEVEL definitions
 * trace: 0
 * debug: 1
 * info: 2
 * warning: 3
 * error: 4
 * fatal: 5
 */

namespace pax { namespace log {
/*! Logging severity levels
 *
 * Either numeric value or string can be used to define loglevel in
 * CMake and environment variables
 */
enum severity_level {
    trace   = 0, /**< displays every available log message */
    debug   = 1, /**< displays most log messages necessary for debugging internals */
    info    = 2, /**< informational messages about setup and what is going on*/
    warning = 3, /**< something is not right but operation can continue */
    error   = 4, /**< something has gone wrong */
    fatal   = 5, /**< something has gone horribly wrong */
    off     = 6, /**< logging is turned off */
};

/*! Logging info structure
 *
 * Information needed to create a log entry is fully contained in the
 * logging_info structure.
 */
struct PAX_API logging_info
{
    logging_info() : verbosity(pax::log::off) {}
    logging_info(const boost::posix_time::ptime& time_,
        const pax::log::severity_level& verbosity_,
        const std::string& file_,
        const unsigned int& line_,
        const std::string& component_,
        const boost::thread::id& thread_id_)
        : time(time_)
        , verbosity(verbosity_)
        , file(file_)
        , line(line_)
        , component(component_)
        , thread_id(thread_id_)
    { /* nop */
    }

    boost::posix_time::ptime time;
    pax::log::severity_level verbosity;
    std::string file;
    unsigned int line;
    std::string component;
    boost::thread::id thread_id;
    std::string message;
};

/*! Set the global log level
 *
 * The global log level gets applied before the specific log level.
 * So, if the global log level is 'info', no logger can can print
 * messages at level 'debug' or below.
 */
PAX_API void set_log_level(pax::log::severity_level level);

/*! Set the log level for the console logger (if defined).
 *
 * Short-hand for `set_logger_level("console", level);`
 */
PAX_API void set_console_level(pax::log::severity_level level);

/*! Set the log level for the file logger (if defined)
 *
 * Short-hand for `set_logger_level("file", level);`
 */
PAX_API void set_file_level(pax::log::severity_level level);

/*! Set the log level for any specific logger.
 *
 * \param logger Name of the logger
 * \param level New log level for this logger.
 *
 * \throws pax::key_error if \p logger was not defined
 */
PAX_API void set_logger_level(const std::string& logger, pax::log::severity_level level);
}} // namespace pax::log

//! \cond
//! Internal logging macro to be used in other macros
#define _PAX_LOG_INTERNAL(component, level) \
    pax::_log::log(level, __FILE__, __LINE__, component, boost::this_thread::get_id())
//! \endcond

// macro-style logging (compile-time determined)
#if PAX_LOG_MIN_LEVEL < 1
#    define PAX_LOG_TRACE(component, message) \
        _PAX_LOG_INTERNAL(component, pax::log::trace) << message;
#else
#    define PAX_LOG_TRACE(component, message)
#endif

#if PAX_LOG_MIN_LEVEL < 2
#    define PAX_LOG_DEBUG(component, message) \
        _PAX_LOG_INTERNAL(component, pax::log::debug) << message;
#else
#    define PAX_LOG_DEBUG(component, message)
#endif

#if PAX_LOG_MIN_LEVEL < 3
#    define PAX_LOG_INFO(component, message) \
        _PAX_LOG_INTERNAL(component, pax::log::info) << message;
#else
#    define PAX_LOG_INFO(component, message)
#endif

#if PAX_LOG_MIN_LEVEL < 4
#    define PAX_LOG_WARNING(component, message) \
        _PAX_LOG_INTERNAL(component, pax::log::warning) << message;
#else
#    define PAX_LOG_WARNING(component, message)
#endif

#if PAX_LOG_MIN_LEVEL < 5
#    define PAX_LOG_ERROR(component, message) \
        _PAX_LOG_INTERNAL(component, pax::log::error) << message;
#else
#    define PAX_LOG_ERROR(component, message)
#endif

#if PAX_LOG_MIN_LEVEL < 6
#    define PAX_LOG_FATAL(component, message) \
        _PAX_LOG_INTERNAL(component, pax::log::fatal) << message;
#else
#    define PAX_LOG_FATAL(component, message)
#endif

#define RFNOC_LOG_TRACE(message) PAX_LOG_TRACE(this->get_unique_id(), message)
#define RFNOC_LOG_DEBUG(message) PAX_LOG_DEBUG(this->get_unique_id(), message)
#define RFNOC_LOG_INFO(message) PAX_LOG_INFO(this->get_unique_id(), message)
#define RFNOC_LOG_WARNING(message) PAX_LOG_WARNING(this->get_unique_id(), message)
#define RFNOC_LOG_ERROR(message) PAX_LOG_ERROR(this->get_unique_id(), message)
#define RFNOC_LOG_FATAL(message) PAX_LOG_FATAL(this->get_unique_id(), message)

#ifndef PAX_LOG_FASTPATH_DISABLE
//! Extra-fast logging macro for when speed matters.
// No metadata is tracked. Only the message is displayed. This does not go
// through the regular backends. Mostly used for printing the UOSDL characters
// during streaming.
#    define PAX_LOG_FASTPATH(message) pax::_log::log_fastpath(message);
#else
#    define PAX_LOG_FASTPATH(message)
#endif

// iostream-style logging
#define PAX_LOGGER_TRACE(component) _PAX_LOG_INTERNAL(component, pax::log::trace)
#define PAX_LOGGER_DEBUG(component) _PAX_LOG_INTERNAL(component, pax::log::debug)
#define PAX_LOGGER_INFO(component) _PAX_LOG_INTERNAL(component, pax::log::info)
#define PAX_LOGGER_WARNING(component) _PAX_LOG_INTERNAL(component, pax::log::warning)
#define PAX_LOGGER_ERROR(component) _PAX_LOG_INTERNAL(component, pax::log::error)
#define PAX_LOGGER_FATAL(component) _PAX_LOG_INTERNAL(component, pax::log::fatal)


#if defined(__GNUG__)
//! Helpful debug tool to print site info
#    define PAX_HERE()            \
        PAX_LOGGER_DEBUG("DEBUG") \
            << __FILE__ << ":" << __LINE__ << " (" << __PRETTY_FUNCTION__ << ")";
#else
//! Helpful debug tool to print site info
#    define PAX_HERE() PAX_LOGGER_DEBUG("DEBUG") << __FILE__ << ":" << __LINE__;
#endif

//! Helpful debug tool to print a variable
#define PAX_VAR(var) PAX_LOGGER_DEBUG("DEBUG") << #var << " = " << var;

//! Helpful debug tool to print a variable in hex
#define PAX_HEX(var)                                                              \
    PAX_LOGGER_DEBUG("DEBUG") << #var << " = 0x" << std::hex << std::setfill('0') \
                              << std::setw(8) << var << std::dec;

//! \cond
namespace pax {
namespace _log {

//! Fastpath logging
void PAX_API log_fastpath(const std::string&);

//! Internal logging object (called by PAX_LOG* macros)
class PAX_API log
{
public:
    log(const pax::log::severity_level verbosity,
        const std::string& file,
        const unsigned int line,
        const std::string& component,
        const boost::thread::id thread_id);

    ~log(void);

// Macro for overloading insertion operators to avoid costly
// conversion of types if not logging.
#define INSERTION_OVERLOAD(x) \
    log& operator<<(x)        \
    {                         \
        if (_log_it) {        \
            _ss << val;       \
        }                     \
        return *this;         \
    }

    // General insertion overload
    template <typename T>
    INSERTION_OVERLOAD(T val)

    // Insertion overloads for std::ostream manipulators
    INSERTION_OVERLOAD(std::ostream& (*val)(std::ostream&))
        INSERTION_OVERLOAD(std::ios& (*val)(std::ios&))
            INSERTION_OVERLOAD(std::ios_base& (*val)(std::ios_base&))

                private : pax::log::logging_info _log_info;
    std::ostringstream _ss;
    const bool _log_it;
};

} // namespace _log
//! \endcond
} /* namespace pax */



//
// Copyright 2010-2016 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

/*!
 * The ABI version string that the client application builds against.
 * Call get_abi_string() to check this against the library build.
 * The format is oldest API compatible release - ABI compat number.
 * The compatibility number allows pre-release ABI to be versioned.
 */
#define PAX_VERSION_ABI_STRING "2.1.0"

/*!
 * A macro to check PAX version at compile-time.
 * The value of this macro is MAJOR * 1000000 + API * 10000 + ABI * 100 + PATCH
 * (e.g., for PAX 3.10.0.1 this is 3100001).
 */
#define PAX_VERSION "2.1.0"

#ifdef __cplusplus
#include <config.h>
#include <string>

namespace pax{

    //! Get the version string (dotted version number + build info)
    PAX_API std::string get_version_string(void);

    //! Get the ABI compatibility string for this build of the library
    PAX_API std::string get_abi_string(void);

    //! Get the component string
    PAX_API std::string get_component(void);

} //namespace uhd
#endif
