#include "eeprom.hpp"
#include <fw_common.h>
#include <string>
#include <boost/algorithm/hex.hpp>
pax::eeprom::sptr PAX_API pax::eeprom::make(pax::i2c_iface::sptr iface)
{
    return sptr(new eeprom_imple(iface));
}



pax::eeprom_imple::eeprom_imple(pax::i2c_iface::sptr iface):iface(iface)
{
    //iface->write_i2c(0x50,byte_vector_t(1, boost::uint8_t(0)));
    read_info();
    analyze();
}

void  pax::eeprom_imple::read_info(){
    buff=iface->read_eeprom(USRP2_I2C_DEV_EEPROM,0,128);
    return;
}

void  pax::eeprom_imple::set_info(const board_info &binfo){
    info.board_mfg.resize(binfo.board_mfg.size());
    info.part.resize(binfo.part.size());
    info.product_name.resize(binfo.product_name.size());
    info.serial.resize(binfo.serial.size());
    info.ad_ref_clk.resize(binfo.ad_ref_clk.size());

    std::copy(binfo.board_mfg.begin(),binfo.board_mfg.end(),info.board_mfg.begin());
    std::copy(binfo.part.begin(),binfo.part.end(),info.part.begin());
    std::copy(binfo.product_name.begin(),binfo.product_name.end(),info.product_name.begin());
    std::copy(binfo.serial.begin(),binfo.serial.end(),info.serial.begin());
    std::copy(binfo.ad_ref_clk.begin(),binfo.ad_ref_clk.end(),info.ad_ref_clk.begin());

    //NOTE: Date is not copied, analyze_write will set it
    analyze_write();
}


void pax::eeprom_imple::write_info(){
    iface->write_eeprom(USRP2_I2C_DEV_EEPROM,0,buff);
}

void pax::eeprom_imple::analyze()
{
    //  boost::gregorian::date current_date(boost::gregorian::day_clock::local_day());
    info.product_date= boost::gregorian::date (1996,1,1);

    // current_date=current_date+c_date;
    if(buff[0]!=1)
    {
        return;
    }

    //checksum
    int offset=buff[3]*8;
    int end_check=buff[offset+1]*8+offset;
    boost::uint8_t ch=0;
    for(int i=offset;i<end_check;i++)
        ch+=buff[i];
    // end
    size_t len=buff[buff[3]*8+6]&0x3F;
    info.board_mfg.resize(len);

    offset=buff[3]*8+3;
    boost::uint32_t time=(((boost::uint32_t)buff[offset]) \
                          +(((boost::uint32_t)buff[offset+1])<<8) \
            +(((boost::uint32_t)buff[offset+2])<<16)) \
            /60/24;
    boost::gregorian::days c_date(time);
    info.product_date=info.product_date+c_date;
    offset=buff[3]*8+7;
    std::copy(buff.begin()+offset,buff.begin()+len+offset,info.board_mfg.begin());
    offset=offset+len;
    len=buff[offset]&0x3f;
    offset++;
    info.product_name.resize(len);
    std::copy(buff.begin()+offset,buff.begin()+len+offset,info.product_name.begin());
    offset=offset+len;
    len=buff[offset]&0x3f;
    offset++;
    info.serial.resize(len);
    std::copy(buff.begin()+offset,buff.begin()+len+offset,info.serial.begin());
    offset=offset+len;
    len=buff[offset]&0x3f;
    offset++;
    info.part.resize(len);
    std::copy(buff.begin()+offset,buff.begin()+len+offset,info.part.begin());

    //////////PH
    offset=offset+len;
    len=buff[offset]&0x3f;
    offset++;
    info.ad_ref_clk.resize(len);
    std::copy(buff.begin()+offset,buff.begin()+len+offset,info.ad_ref_clk.begin());
    //////////PH

    return;
}

std::string pax::eeprom_imple::get_name()
{
    return "";
}

void pax::eeprom_imple::analyze_write()
{
    buff.resize(128);
    memset(buff.data(),0,buff.size());
    buff[0]=1;
    buff[3]=1;
    buff[7]=0xfe;
    buff[8]=1;

    buff[10]=0x19;
    boost::gregorian::date base_date(1996,1,1);
    boost::gregorian::date current_date(boost::gregorian::day_clock::local_day());
    boost::gregorian::days time=current_date-base_date;
    boost::uint32_t mins=time.days()*24*60;

    buff[11]=mins&0xff;
    buff[12]=(mins>>8)&0xff;
    buff[13]=(mins>>16)&0xff;

    buff[14]=info.board_mfg.size()|0xC0;
    std::copy(info.board_mfg.begin(),info.board_mfg.end(),buff.begin()+15);
    int offset=14+info.board_mfg.size()+1;
    buff[offset]=info.product_name.size()|0xC0;
    offset++;
    std::copy(info.product_name.begin(),info.product_name.end(),buff.begin()+offset);


    offset=offset+info.product_name.size();
    buff[offset]=info.serial.size()|0xC0;
    offset++;
    std::copy(info.serial.begin(),info.serial.end(),buff.begin()+offset);


    offset=offset+info.serial.size();
    buff[offset]=info.part.size()|0xC0;
    offset++;
    std::copy(info.part.begin(),info.part.end(),buff.begin()+offset);

    offset=offset+info.part.size();

    //////////PH
    buff[offset]=info.ad_ref_clk.size()|0xC0;
    offset++;
    std::copy(info.ad_ref_clk.begin(),info.ad_ref_clk.end(),buff.begin()+offset);



    offset=offset+info.ad_ref_clk.size();
    //////////PH

    buff[offset]=0xc0;
    buff[offset+1]=0xc1;
    buff[9]=(offset+3-8+7)/8;
    int end_check=buff[9]*8+8-1;
    boost::uint8_t ch=0;
    for(int i=8;i<end_check;i++)
        ch+=buff[i];
    buff[end_check]=-ch;
    return ;
}


void pax::eeprom_imple::set_ip_addr(eth_ip_addr_t ip_adr){
    if(ip_adr.addr[0] != 193 && ip_adr.addr[1] != 168 && ip_adr.addr[2] != 10)
        throw value_error("ip addr must be type of 193.168.10.x");
    if(ip_adr.addr[3] == 255 || ip_adr.addr[3] == 0)
        throw value_error("ip addr must be difrent with 192.168.10.0 , 192.168.10.255");

    byte_vector_t temp;
    for(int i=0;i<4;i++)
        temp.push_back(ip_adr.addr[i]);
    iface->write_eeprom(USRP2_I2C_DEV_EEPROM,USRP2_EE_MBOARD_IP_ADDR,temp);
}
void  pax::eeprom_imple::set_mac_addr(eth_mac_addr_t mac_adr){
    byte_vector_t temp;
    for(int i=0;i<6;i++)
        temp.push_back(mac_adr.addr[i]);
    iface->write_eeprom(USRP2_I2C_DEV_EEPROM,USRP2_EE_MBOARD_MAC_ADDR,temp);
}
pax::eth_ip_addr_t  pax::eeprom_imple::get_ip_addr(){
    byte_vector_t temp;
    eth_ip_addr_t out;
    temp = iface->read_eeprom(USRP2_I2C_DEV_EEPROM,USRP2_EE_MBOARD_IP_ADDR,4);
    for(int i=0;i<4;i++)
        out.addr[i] = temp[i];
    return out;
}
pax::eth_mac_addr_t pax::eeprom_imple::get_mac_addr(){
    byte_vector_t temp;
    eth_mac_addr_t out;
    temp = iface->read_eeprom(USRP2_I2C_DEV_EEPROM,USRP2_EE_MBOARD_MAC_ADDR,6);
    for(int i=0;i<6;i++)
        out.addr[i] = temp[i];
    return out;

}

    void pax::eth_mac_addr_t::set_addr(std::string _addr){
        std::vector<std::string> strs;
        boost::split(strs,_addr,boost::is_any_of(":"));
        if(strs.size() != 6)
            throw value_error("wrong mac addr input set");
        for(uint32_t i=0;i<strs.size();i++){
            if(strs[i].length()!=2)
                throw value_error("input mac addr should be like this 00:50:C2:85:3F:FF format");
            boost::cnv::cstream ccnv;

            try{  addr[i] = boost::convert<int>(strs[i], ccnv(std::hex)(std::skipws)).value(); }
            catch(...){throw value_error("wrong hexadecimal input");}

        }
    }
    void  pax::eth_ip_addr_t::set_addr(std::string _addr){
        std::vector<std::string> strs;
        boost::split(strs,_addr,boost::is_any_of("."));
        if(strs.size() != 4)
            throw value_error("wrong ip addr input set shpuld be in this format 192.168.10.3");
        for(uint32_t i=0;i<strs.size();i++){
            boost::cnv::cstream ccnv;

            try{  addr[i] = boost::convert<int>(strs[i], ccnv(std::dec)(std::skipws)).value(); }
            catch(...){throw value_error("wrong hexadecimal input");}

        }
    }





    std::string  pax::eth_ip_addr_t::get_addr(){
        std::string out;boost::cnv::wstream ccnv;
        ccnv(boost::cnv::parameter::base = boost::cnv::base::dec);
        for(int i=0;i<4;i++){
            out+=(boost::convert<std::string>(addr[i], ccnv)).value_or( "XXX");
            if(i!=3)
                out+=".";
        }
        return out;
    }

    std::string pax::eth_mac_addr_t::get_addr(){
        std::string out;

        boost::cnv::wstream ccnv;
        ccnv(boost::cnv::parameter::base = boost::cnv::base::hex)
            (boost::cnv::parameter::uppercase = true)    ;

        for(int i=0;i<6;i++){
            out+=(boost::convert<std::string>(addr[i], ccnv)).value_or("XX");
            if(i!=5)
                out+=":";
        }
        return out;

    }



    bool  pax::eth_ip_addr_t::operator == (const pax::eth_ip_addr_t &b){
        for(int i=0;i<4;i++){
         if(addr[i]!=b.addr[i])
             return false;
        }
        return true;
    }

    bool pax::eth_mac_addr_t::operator == (const pax::eth_mac_addr_t &b){
        for(int i=0;i<6;i++){
         if(addr[i]!=b.addr[i])
             return false;
        }
        return true;
    }
    pax::eth_ip_addr_t::eth_ip_addr_t(std::string _addr){set_addr(_addr);}
    pax::eth_mac_addr_t::eth_mac_addr_t(std::string _addr){set_addr(_addr);}


