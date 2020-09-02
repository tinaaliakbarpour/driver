#ifndef __EEPROM__
#define __EEPROM__

#include <pax_iface.h>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/convert.hpp>
#include <boost/convert/stream.hpp>

namespace pax {


class PAX_API eth_mac_addr_t{
public:
    eth_mac_addr_t (std::string _addr = "00:50:C2:85:3F:FF");
    void set_addr(std::string _addr = "00:50:C2:85:3F:FF");
    std::string get_addr();
    bool operator == (const eth_mac_addr_t &b);

    uint8_t addr[6];
};

class PAX_API eth_ip_addr_t{
public:
    eth_ip_addr_t (std::string _addr = "192.168.10.3");
    void set_addr(std::string _addr= "192.168.10.3");
    std::string get_addr();
    bool operator == (const eth_ip_addr_t &b);

  uint8_t	addr[4];
};







class eeprom
{
public:
    typedef  boost::shared_ptr<eeprom> sptr;
    typedef struct {
        //  boost::date_time product_date;
        std::string board_mfg;
        std::string product_name;
        std::string serial;
        std::string part;
        boost::gregorian::date product_date;
        std::string ad_ref_clk;
    } board_info;
    board_info info;

    PAX_API static sptr make(pax::i2c_iface::sptr iface);
    PAX_API virtual std::string get_name()=0;
    PAX_API virtual void read_info()=0;
    PAX_API virtual void set_info(const board_info &binfo)=0;
    PAX_API virtual void write_info()=0;

    PAX_API virtual void set_ip_addr(eth_ip_addr_t)=0;
    PAX_API virtual void set_mac_addr(eth_mac_addr_t)=0;
    PAX_API virtual eth_ip_addr_t get_ip_addr()=0;
    PAX_API virtual eth_mac_addr_t get_mac_addr()=0;



};




class eeprom_imple:public eeprom
{
public:
    eeprom_imple(pax::i2c_iface::sptr iface);
    pax::i2c_iface::sptr iface;
    pax::byte_vector_t buff;

    std::string get_name();
    void read_info();
    void set_info(const board_info &binfo);
    void write_info();
    void analyze();
    void analyze_write();

    void set_ip_addr(eth_ip_addr_t);
    void set_mac_addr(eth_mac_addr_t);
    eth_ip_addr_t get_ip_addr();
    eth_mac_addr_t get_mac_addr();
private:



};

}
#endif
