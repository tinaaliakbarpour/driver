//
// Copyright 2010-2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <config.h>
#include <boost/current_function.hpp>
#include <stdexcept>
#include <string>

/*!
 * Define common exceptions used throughout the code:
 *
 * - The python built-in exceptions were used as inspiration.
 * - Exceptions inherit from std::exception to provide what().
 * - Exceptions inherit from pax::exception to provide code().
 *
 * The code() provides an error code which allows the application
 * the option of printing a cryptic error message from the 1990s.
 *
 * The dynamic_clone() and dynamic_throw() methods allow us to:
 * catch an exception by dynamic type (i.e. derived class), save it,
 * and later rethrow it, knowing only the static type (i.e. base class),
 * and then finally to catch it again using the derived type.
 *
 * http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2006/n2106.html
 */
namespace pax {

/*! Base class of all PAX-specific exceptions.
 */
struct PAX_API exception : std::runtime_error
{
    exception(const std::string& what);
    virtual unsigned code(void) const            = 0;
    virtual exception* dynamic_clone(void) const = 0;
    virtual void dynamic_throw(void) const       = 0;
};

/*! Raised when an assert statement fails.
 *
 * This includes our own assertion macros, such as PAX_ASSERT_THROW().
 */
struct PAX_API assertion_error : exception
{
    assertion_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual assertion_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! The base class for exceptions that are raised when a key or index is
 * invalid.
 */
struct PAX_API lookup_error : exception
{
    lookup_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual lookup_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a sequence index is out of range.
 */
struct PAX_API index_error : lookup_error
{
    index_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual index_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a dictionary-like object is trying to be indexed by an
 *  invalid key.
 *
 * This includes the property tree.
 */
struct PAX_API key_error : lookup_error
{
    key_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual key_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when an operation or function is executed with a value of
 *  incorrect type.
 *
 * This might occur when values are being passed around as strings, but the
 * underlying code will need convert to a native type.
 */
struct PAX_API type_error : exception
{
    type_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual type_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when an operation or function receives an argument that has the
 * right type but an inappropriate or invalid value, and no other exception
 * is more specific.
 */
struct PAX_API value_error : exception
{
    value_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual value_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a value is inappropriate because it can't be narrowed as
 * required.
 *
 * Mostly raised by pax::narrow()
 */
struct PAX_API narrowing_error : value_error
{
    narrowing_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual narrowing_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when an error is detected that doesn't fall into any of the
 * categories.
 */
struct PAX_API runtime_error : exception
{
    runtime_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual runtime_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when an error occurs during a USB transaction.
 */
struct PAX_API usb_error : runtime_error
{
    int _code;
    usb_error(int code, const std::string& what);
    virtual unsigned code(void) const
    {
        return _code;
    };
    virtual usb_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a function is stubbed out but doesn't actually do anything
 * useful.
 */
struct PAX_API not_implemented_error : runtime_error
{
    not_implemented_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual not_implemented_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a resource is being accessed without having the appropriate
 * permissions.
 */
struct PAX_API access_error : runtime_error
{
    access_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual access_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Base class for errors that occur outside of PAX.
 */
struct PAX_API environment_error : exception
{
    environment_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual environment_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when an I/O operation fails for an I/O-related reason.
 */
struct PAX_API io_error : environment_error
{
    io_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual io_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a function returns a system-related error.
 */
struct PAX_API os_error : environment_error
{
    os_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual os_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! \deprecated
 */
struct PAX_API system_error : exception
{
    system_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual system_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Raised when a parser encounters a syntax error.
 *
 * Within PAX, this is limited to Noc-Script execution.
 */
struct PAX_API syntax_error : exception
{
    syntax_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual syntax_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Base class for RFNoC-related errors
 */
struct PAX_API rfnoc_error : exception
{
    rfnoc_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual rfnoc_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Gets thrown when a transaction with an RFNoC block fails (IO error)
 */
struct PAX_API op_failed : rfnoc_error
{
    op_failed(const std::string& what);
    virtual unsigned code(void) const;
    virtual op_failed* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Gets thrown when a transaction with an RFNoC block times out (e.g., no ACK
 *  received)
 */
struct PAX_API op_timeout : rfnoc_error
{
    op_timeout(const std::string& what);
    virtual unsigned code(void) const;
    virtual op_timeout* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Gets thrown when a transaction with an RFNoC yields a sequence error
 */
struct PAX_API op_seqerr : rfnoc_error
{
    op_seqerr(const std::string& what);
    virtual unsigned code(void) const;
    virtual op_seqerr* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Gets thrown when a transaction with an RFNoC yields a time error (late command)
 */
struct PAX_API op_timerr : rfnoc_error
{
    op_timerr(const std::string& what);
    virtual unsigned code(void) const;
    virtual op_timerr* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Gets thrown when a property resolution fails
 */
struct PAX_API resolve_error : rfnoc_error
{
    resolve_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual resolve_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*! Gets thrown when there is a routing-related failure in RFNoC
 */
struct PAX_API routing_error : rfnoc_error
{
    routing_error(const std::string& what);
    virtual unsigned code(void) const;
    virtual routing_error* dynamic_clone(void) const;
    virtual void dynamic_throw(void) const;
};

/*!
 * Create a formatted string with throw-site information.
 * Fills in the function name, file name, and line number.
 * \param what the std::exception message
 * \return the formatted exception message
 */
#define PAX_THROW_SITE_INFO(what)                                                        \
    std::string(std::string(what) + "\n" + "  in " + std::string(BOOST_CURRENT_FUNCTION) \
                + "\n" + "  at " + std::string(__FILE__) + ":"                           \
                + BOOST_STRINGIZE(__LINE__) + "\n")

/*!
 * Throws an invalid code path exception with throw-site information.
 * Use this macro in places that code execution is not supposed to go.
 */
#define PAX_THROW_INVALID_CODE_PATH() \
    throw pax::system_error(PAX_THROW_SITE_INFO("invalid code path"))

/*!
 * Assert the result of the code evaluation.
 * If the code evaluates to false, throw an assertion error.
 * \param code the code that resolved to a boolean
 */
#define PAX_ASSERT_THROW(code)                                      \
    {                                                               \
        if (not(code))                                              \
            throw pax::assertion_error(PAX_THROW_SITE_INFO(#code)); \
    }

} // namespace pax
