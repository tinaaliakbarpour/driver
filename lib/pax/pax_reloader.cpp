#include <pax_reload.hpp>
#include <byteswap.hpp>

boost::uint32_t pax::pax_reload::bit_reverse(boost::uint32_t x)
{
  x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
  x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
  x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
  x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));
  return((x >> 16) | (x << 16));
}

boost::uint16_t pax::pax_reload::bit_reverse(boost::uint16_t in){
    boost::uint32_t x = static_cast<boost::uint32_t>(in);
    boost::uint32_t y = (bit_reverse(x))>>16;
    return static_cast<boost::uint16_t>(y);
}
boost::uint8_t pax::pax_reload::bit_reverse(boost::uint8_t in){
    boost::uint32_t x = static_cast<boost::uint32_t>(in);
    boost::uint32_t y = (bit_reverse(x))>>24;
    return static_cast<boost::uint8_t>(y);
}
boost::uint16_t pax::pax_reload::byte_reverse(boost::uint16_t x){
    return pax::byteswap(x);
}
boost::uint32_t pax::pax_reload::byte_reverse(boost::uint32_t x){
        return pax::byteswap(x);
}


pax::byte_vector_t& pax::pax_reload::make_standard_file(const std::string& addr_file_path){
    static byte_vector_t result;
    return result;
}

pax::pax_reload::supported_file_t pax::pax_reload::detect_file_type(const std::string& addr_file_path){
    if( addr_file_path.find(".mcs") >= 0){

    } else if (addr_file_path.find(".bit") >= 0) {

    } else {
        throw (type_error("please enter bit file or mcs file for programming"));
    }
}



