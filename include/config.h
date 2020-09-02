#ifndef CONFIG_H
#define CONFIG_H

#include <boost/config.hpp>

#ifdef BOOST_MSVC
// suppress warnings
//# pragma warning(push)
//# pragma warning(disable: 4511) // copy constructor can't not be generated
//# pragma warning(disable: 4512) // assignment operator can't not be generated
//# pragma warning(disable: 4100) // unreferenced formal parameter
//# pragma warning(disable: 4996) // <symbol> was declared deprecated
# pragma warning(disable: 4355) // 'this' : used in base member initializer list
//# pragma warning(disable: 4706) // assignment within conditional expression
# pragma warning(disable: 4251) // class 'A<T>' needs to have dll-interface to be used by clients of class 'B'
//# pragma warning(disable: 4127) // conditional expression is constant
//# pragma warning(disable: 4290) // C++ exception specification ignored except to ...
//# pragma warning(disable: 4180) // qualifier applied to function type has no meaning; ignored
# pragma warning(disable: 4275) // non dll-interface class ... used as base for dll-interface class ...
//# pragma warning(disable: 4267) // 'var' : conversion from 'size_t' to 'type', possible loss of data
//# pragma warning(disable: 4511) // 'class' : copy constructor could not be generated
//# pragma warning(disable: 4250) // 'class' : inherits 'method' via dominance
# pragma warning(disable: 4200) // nonstandard extension used : zero-sized array in struct/union

// define logical operators
#include <ciso646>

// define ssize_t
#include <cstddef>
typedef ptrdiff_t ssize_t;

#endif //BOOST_MSVC

//define cross platform attribute macros
#if defined(BOOST_MSVC)
#define PAX_EXPORT         __declspec(dllexport)
#define PAX_IMPORT         __declspec(dllimport)
#define PAX_INLINE         __forceinline
#define PAX_DEPRECATED     __declspec(deprecated)
#define PAX_ALIGNED(x)     __declspec(align(x))
#define PAX_UNUSED(x)      x
#elif defined(__MINGW32__)
#define PAX_EXPORT         __declspec(dllexport)
#define PAX_IMPORT         __declspec(dllimport)
#define PAX_INLINE         inline
#define PAX_DEPRECATED     __declspec(deprecated)
#define PAX_ALIGNED(x)     __declspec(align(x))
#define PAX_UNUSED(x)      x __attribute__((unused))
#elif defined(__GNUG__) && __GNUG__ >= 4
#define PAX_EXPORT            __attribute__((visibility("default")))
#define PAX_IMPORT            __attribute__((visibility("default")))
#define PAX_INLINE         inline __attribute__((always_inline))
#define PAX_DEPRECATED     __attribute__((deprecated))
#define PAX_ALIGNED(x)     __attribute__((aligned(x)))
#define PAX_UNUSED(x)      x __attribute__((unused))
#else
#define PAX_EXPORT
#define PAX_IMPORT
#define PAX_INLINE         inline
#define PAX_DEPRECATED
#define PAX_ALIGNED(x)
#define PAX_UNUSED(x)      x
#endif

// Define API declaration macro
#ifdef PAX_DLL_EXPORTS
#define PAX_API PAX_EXPORT
#else
#define PAX_API PAX_IMPORT
#endif // PAX_DLL_EXPORTS

// Platform defines for conditional parts of headers:
// Taken from boost/config/select_platform_config.hpp,
// however, we define macros, not strings for platforms.
#if defined(linux) || defined(__linux) || defined(__linux__)
#define PAX_PLATFORM_LINUX
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define PAX_PLATFORM_WIN32

#include <stdint.h>

#elif defined(macintosh) || defined(__APPLE__) || defined(__APPLE_CC__)
#define PAX_PLATFORM_MACOS
#elif defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
#define PAX_PLATFORM_BSD
#endif

// Define 'stringize' preprocessor macros. The stringize macro, XSTR, takes
// variable arguments so that it can deal with strings that contain commas.
// There are two different versions because MSVC handles this syntax a bit
// differently than other compilers.
#if defined(BOOST_MSVC)
#define XSTR(x,...) #x
#else
#define XSTR(x...) #x
#endif

#define STR(x) XSTR(x)

#endif // CONFIG_H

