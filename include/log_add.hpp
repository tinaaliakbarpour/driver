//
// Copyright 2017 Ettus Research (National Instruments Corp.)
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

// Note: Including this file requires C++11 features enabled.

#pragma once

#include <config.h>
#include <log.hpp>
#include <functional>

namespace pax { namespace log {

/*! Logging function type
 *
 * Every logging_backend has to define a function with this signature.
 * Can be added to the logging core.
 */
typedef std::function<void(const pax::log::logging_info&)> log_fn_t;

/*! Add logging backend to the log system
 *
 * \param key Identifies the logging backend in the logging core
 * \param logger_fn function which actually logs messages to this backend
 */
PAX_API void add_logger(const std::string& key, log_fn_t logger_fn);
}} /* namespace pax::log */
