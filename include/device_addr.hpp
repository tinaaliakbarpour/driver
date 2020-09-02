#ifndef DEVICE_ADDR_HPP
#define DEVICE_ADDR_HPP
#include "dict.hpp"
namespace pax {

class PAX_API device_addr_t : public dict<std::string, std::string>{
public:
    /*!
     * Create a device address from an args string.
     * \param args the arguments string
     */
    device_addr_t(const std::string &args = "");

    /*!
     * Convert a device address into a pretty print string.
     * \return a printable string representing the device address
     */
    std::string to_pp_string(void) const;

    /*!
     * Convert the device address into an args string.
     * The args string contains delimiter symbols.
     * \return a string with delimiter markup
     */
    std::string to_string(void) const;

    /*!
     * Lexically cast a parameter to the specified type,
     * or use the default value if the key is not found.
     * \param key the key as one of the address parameters
     * \param def the value to use when key is not present
     * \return the casted value as type T or the default
     * \throw error when the parameter cannot be casted
     */
    template <typename T> T cast(const std::string &key, const T &def) const{
        if (not this->has_key(key)) return def;
        try{
            return boost::lexical_cast<T>((*this)[key]);
        }
        catch(const boost::bad_lexical_cast &){
            throw std::runtime_error("cannot cast " + key + " = " + (*this)[key]);
        }
    }
};

//! A typedef for a vector of device addresses
typedef std::vector<device_addr_t> device_addrs_t;

//! Separate an indexed device address into a vector of device addresses
PAX_API device_addrs_t separate_device_addr(const device_addr_t &dev_addr);

//! Combine a vector of device addresses into an indexed device address
PAX_API device_addr_t combine_device_addrs(const device_addrs_t &dev_addrs);

} //namespace pax

#endif /* INCLUDED_PAX_TYPES_DEVICE_ADDR_HPP */


