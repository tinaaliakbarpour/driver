//
// Copyright 2011-2013 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <serial.hpp>
#include <boost/thread.hpp> //for sleeping
#include <boost/assign/list_of.hpp>
#include <log.hpp>
using namespace pax;




void flash_iface::read_flash(uint32_t _addr,flash_vec_t& out,uint32_t num){
    if(num==0)
        return;
    addr=_addr;
    init_private_filed(out,num,READ);
    transact(READ_CMD);
    out=out_vec;
}
void flash_iface::write_flash(uint32_t _addr,const flash_vec_t& out){
    if(out.size()==0)
        return;
    addr=_addr;
    init_private_filed(out,0,WRITE);
    transact(WRITE_CMD);
}
void flash_iface::erase_flash(uint32_t _addr,uint32_t num){
    if(num==0)
        return;
   addr=_addr;
   sample=num;
   flash_vec_t null_vec;
   init_private_filed(null_vec,num,ERASE);
   transact(ERASE_CMD);
}
void flash_iface::read_otp_flash(uint32_t _addr,flash_vec_t& out,uint32_t num,flash::flash_OTP_section_reg_t read_section){

    save_sec=read_section;out.clear();
    addr=_addr+otp_section_addr_map(save_sec).from+_addr;
    init_private_filed(out,num,READ);
    transact(READ_OTP_CMD);
    out=out_vec;
}

void flash_iface::write_otp_flash(uint32_t _addr,const flash_vec_t& out,flash::flash_OTP_section_reg_t write_section){
    save_sec=write_section;
    addr=_addr+otp_section_addr_map(save_sec).from;
    sample=out.size();
    init_private_filed(out,0,WRITE);
    transact(WRITE_OTP_CMD);
}
void flash_iface::erase_otp_flash(flash::flash_OTP_section_reg_t erase_section){
    save_sec=erase_section;flash_vec_t null_vec;
    addr=otp_section_addr_map(save_sec).from;sample=1;
    init_private_filed(null_vec,0,ERASE);
    transact(ERASE_OTP_CMD);
}
void flash_iface::lock_otp_flash(flash::flash_OTP_section_reg_t section){
    save_sec=section;sample=1;
    addr=otp_section_addr_map(save_sec).from;
    transact(LOCK_OTP_CMD);
}
flash_vec_t flash_iface::read_uid_flash(){
    transact(READ_UID_CMD);
    return out_vec;
}
boost::uint16_t flash_iface::read_flash(uint32_t _addr){

    flash_vec_t temp_vec;
    read_flash(_addr,temp_vec,1);
    return temp_vec[0];

}
void flash_iface::write_flash(uint32_t _addr,uint32_t _data){
    flash_vec_t temp_vec;temp_vec.push_back(_data);
    write_flash(_addr,temp_vec);
}
void flash_iface::erase_full_flash(){
    transact(ERASE_FULL_CMD);
}




void flash_iface::set_flash_type (flash::flash_type_t type){
    switch(type){
    case flash::SPI_IS25LPxxx:
       PAX_LOGGER_INFO("PAX")<<"flash FOUND : SPI flash IS25LPXXX";
    break;
    case flash::BPI_28F00AP30:
        PAX_LOGGER_INFO("PAX")<<"flash FOUND : BPI flash 28F00AP30";
    break;
    case flash::BPI_S29GL01GS:
       PAX_LOGGER_INFO("PAX")<<"flash FOUND : BPI flash S29GL01GS";
    break;
    default:{throw not_implemented_error("this type of flash not been implemented! ");}
    }
    flash_type=type;
}
flash::flash_type_t flash_iface::get_flash_type (){return flash_type;}


void flash_iface::set_flash_interface (flash::flash_interface_t interface_type){
    switch(interface_type){
    case flash::BPI_FLASH_INTERFACAE_LIMIT:
    case flash::BPI_FLASH_16x:
        flash_interface = interface_type;
    break;
    default:{throw not_implemented_error("this type of flash not been implemented! ");}
    }
}
flash::flash_interface_t flash_iface::get_flash_interface (){
    return flash_interface;
}


void flash_iface::_init_private_field_8x_data_width(const flash_vec_t& _data,uint32_t num,minor_cmd cmd){
    if(cmd==WRITE){
        num=_data.size();
    }
    flash_vec_t temp=divide_number(num,MAX_NUMBER_OF_BUFFER_IN_SPIF_FRIMWARE);
    uint32_t iterator_number=0;
    for(uint32_t i=0;i<temp.size();i++){
        data_splited<uint8_t,MAX_NUMBER_OF_BUFFER_IN_SPIF_FRIMWARE> tempo;
        tempo.addr=iterator_number+addr;
        tempo.num=temp[i];
        if(cmd==WRITE){
            for(uint32_t j=0;j<tempo.num;j++){
              tempo.data[j]=_data[iterator_number+j];
            }
        }
        iterator_number+=tempo.num;
        data_8.push_back(tempo);
    }
    sample=num;
}

void flash_iface::_init_private_field_16x_data_width(const flash_vec_t& _data,uint32_t num,minor_cmd cmd){
    if(cmd==WRITE){
        num=_data.size();
    }
    flash_vec_t temp=divide_number(num,MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE);
    uint32_t iterator_number=0;
    for(uint32_t i=0;i<temp.size();i++){
        data_splited<uint16_t,MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE> tempo;
        tempo.addr=iterator_number+addr;
        tempo.num=temp[i];
        if(cmd==WRITE){
            for(uint32_t j=0;j<tempo.num;j++){
              tempo.data[j]=_data[iterator_number+j];
            }
        }
        iterator_number+=tempo.num;
        data_16.push_back(tempo);
    }
    sample=num;
}

void flash_iface::init_private_filed(const flash_vec_t& _data,uint32_t num,minor_cmd cmd){
    data_8.clear();data_16.clear();
    _init_private_field_8x_data_width(_data,num,cmd);
    _init_private_field_16x_data_width(_data,num,cmd);
}
std::vector<uint32_t> flash_iface::divide_number_32(uint32_t num,uint32_t divider){
    int m,r;std::vector<uint32_t> out;
    r=num%divider;
    m=num-r;
    while(m>0){
        m -= divider;
        out.push_back(divider);
    }
    if(r){
        out.push_back(r);
    }
    return out;
}

flash_vec_t flash_iface::divide_number(uint32_t num,uint32_t divider){
    int m,r;flash_vec_t out;
    r=num%divider;
    m=num-r;
    while(m>0){
        m -= divider;
        out.push_back(divider);
    }
    if(r){
        out.push_back(r);
    }
    return out;
}

boost::uint32_t flash_iface::otp_section_to_command_mapper(flash::flash_OTP_section_reg_t in){
    switch (in) {
    case flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1:  return 0xFFFF;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_1:    return 0xFFFF - (1<<0x0);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_2:    return 0xFFFF - (1<<0x1);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_3:    return 0xFFFF - (1<<0x2);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_4:    return 0xFFFF - (1<<0x3);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_5:    return 0xFFFF - (1<<0x4);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_6:    return 0xFFFF - (1<<0x5);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_7:    return 0xFFFF - (1<<0x6);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_8:    return 0xFFFF - (1<<0x7);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_9:    return 0xFFFF - (1<<0x8);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_10:   return 0xFFFF - (1<<0x9);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_11:   return 0xFFFF - (1<<0xa);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_12:   return 0xFFFF - (1<<0xb);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_13:   return 0xFFFF - (1<<0xc);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_14:   return 0xFFFF - (1<<0xd);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_15:   return 0xFFFF - (1<<0xe);  break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_16:   return 0xFFFF - (1<<0xf);  break;
    case flash::IS25LPxx_OTP_REG_0x000_TO_0x0ff: return (0x00000>>8); break;
    case flash::IS25LPxx_OTP_REG_0x100_TO_0x1ff: return (0x01000>>8); break;
    case flash::IS25LPxx_OTP_REG_0x200_TO_0x2ff: return (0x02000>>8); break;
    case flash::IS25LPxx_OTP_REG_0x300_TO_0x3ff: return (0x03000>>8); break;
    default: {throw value_error("there is no such a section");} break;
    }
}

flash_iface::addr_bindery flash_iface::otp_section_addr_map(flash::flash_OTP_section_reg_t in){
    switch (in) {
    case flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1:{return addr_bindery(0x0100,0x01FF);}
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_1:{return  addr_bindery(0x8A,0x91);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_2:{return  addr_bindery(0x92,0x99);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_3:{return  addr_bindery(0x9A,0xA1);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_4:{return  addr_bindery(0xA2,0xA9);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_5:{return  addr_bindery(0xAA,0xB1);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_6:{return  addr_bindery(0xB2,0xB9);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_7:{return  addr_bindery(0xBA,0xC1);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_8:{return  addr_bindery(0xC2,0xC9);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_9:{return  addr_bindery(0xCA,0xD1);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_10:{return addr_bindery(0xDA,0xD9);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_11:{return addr_bindery(0xDA,0xE1);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_12:{return addr_bindery(0xEA,0xE9);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_13:{return addr_bindery(0xEA,0xF1);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_14:{return addr_bindery(0xF2,0xF9);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_15:{return addr_bindery(0xFA,0x101);}break;
    case flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_16:{return addr_bindery(0x102,0x109);}break;
    case flash::IS25LPxx_OTP_REG_0x000_TO_0x0ff:{return addr_bindery( 0x000000, 0x0000ff);}break;
    case flash::IS25LPxx_OTP_REG_0x100_TO_0x1ff:{return addr_bindery( 0x001000, 0x0010ff);}break;
    case flash::IS25LPxx_OTP_REG_0x200_TO_0x2ff:{return addr_bindery( 0x002000, 0x0020ff);}break;
    case flash::IS25LPxx_OTP_REG_0x300_TO_0x3ff:{return addr_bindery( 0x003000, 0x0030ff);}break;



    default: {throw value_error("there is no such a section");}
        break;
    }
}

float pax::flash_iface::get_write_Percentage_pass(){
    float _dev=static_cast<float>(it_count);
    switch(flash_type){
    case flash::BPI_28F00AP30: throw not_implemented_error("not implemented for this type"); break;
    case flash::SPI_IS25LPxxx: _dev = static_cast<float>((_dev*MAX_NUMBER_OF_BUFFER_IN_SPIF_FRIMWARE)/(sample+1));  break;
    case flash::BPI_S29GL01GS: _dev = static_cast<float>((_dev*MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE)/(sample+1));  break;
    }
    return (_dev*100);
}


void flash_iface::transact(cmd_base_t cmd){
    out_vec.clear();
    switch(cmd){
    case ERASE_CMD: erase_cmd(); break;
    case WRITE_CMD: write_cmd(); break;
    case READ_CMD:  read_cmd(); break;
    case ERASE_OTP_CMD: erase_otp_cmd(); break;
    case WRITE_OTP_CMD: write_otp_cmd(); break;
    case READ_OTP_CMD: read_otp_cmd(); break;
    case LOCK_OTP_CMD: lock_otp_cmd(); break;
    case READ_UID_CMD: read_uid_cmd(); break;
    case ERASE_FULL_CMD: erase_full_cmd(); break;
    default :{throw not_implemented_error("not implimented for spi IS25LPxxx flash!");}break;
    }
}



void flash_iface::erase_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        throw runtime_error("this part is not implemented");
        break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_erase();
    break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_erase_sector_flash();
        break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::write_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        _28F00AP30_write();
    break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_write();
        break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_write_to_buffer();
        break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::read_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        _28F00AP30_read();
    break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_read();
        break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_read();
        break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::erase_otp_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        throw runtime_error("this part is not implemented");
    break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_erase_otp();
     break;
    case flash::BPI_S29GL01GS:
        throw runtime_error("this part is not implemented");
        break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::write_otp_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        _28F00AP30_write_otp();
        break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_write_otp();
        break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_write_otp();
        break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::read_otp_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        _28F00AP30_read_otp();
     break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_read_otp();
     break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_read_otp();
    break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::lock_otp_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
            _28F00AP30_lock_otp();
        break;
    case flash::SPI_IS25LPxxx:
            _IS25LPxx_lock_otp();
        break;
    case flash::BPI_S29GL01GS:
            _S29GL01GS_lock_otp();
        break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::read_uid_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
         _28F00AP30_read_UID();
    break;
    case flash::SPI_IS25LPxxx:
         _IS25LPxx_read_UID();
    break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_read_uid();
    break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}
void flash_iface::erase_full_cmd(){
    switch(flash_type){
    case flash::BPI_28F00AP30:
        throw runtime_error("this part is not implemented");
        break;
    case flash::SPI_IS25LPxxx:
        _IS25LPxx_chip_erase();
     break;
    case flash::BPI_S29GL01GS:
        _S29GL01GS_chip_erase();
    break;
    default : {throw not_implemented_error("this type of flash not implemented!");}
    }
}





//inline void flash_iface::_bpi_write_reg(boost::uint32_t address,boost::uint16_t data){
//    transact_bpi(address,data,USRP2_WRITE_ONE_BPIFLASH_16X);
//}
inline boost::uint32_t flash_iface::_bpi_read_reg(boost::uint32_t address){
    return   transact_bpi(address,address,USRP2_READ_ONE_BPIFLASH_16X);
}

void flash_iface::_28F00AP30_read(){
    for(uint32_t i=0;i<sample;i++)
        out_vec.push_back( _bpi_read_reg(addr+i, 0));//USRP2_READ_ONE_BPIFLASH_16X
}
void flash_iface::_28F00AP30_write(){
    if(sample == 1 )
        _bpi_write_reg(addr,data_16[0].data[0]);//USRP2_WRITE_ONE_BPIFLASH_16X
    else if(sample == 32)
          _bpi_write_buffer(addr,&data_16[0].data[0]);//USRP2_WRITE_32_BPIFLASH_16X
    else
        throw not_implemented_error("this part is not implemented");
}
void flash_iface::_28F00AP30_erase(){

}
void flash_iface::_28F00AP30_read_otp(){
    if(addr+sample>8+otp_section_addr_map(save_sec).from){
        throw value_error("evrey section of otp have 8 word");
    }
    _bpi_read_reg(0x80,0x90);//USRP2_READ_ONE_BPIFLASH_16X
    for(uint32_t i = addr ; i<addr+sample;i++)
        out_vec.push_back(_bpi_read_reg(i,0));//USRP2_READ_ONE_BPIFLASH_16X
}
void flash_iface::_28F00AP30_write_otp(){
    if(addr+sample>8+otp_section_addr_map(save_sec).from){
        throw value_error("evrey section of otp have 8 word");
    }
    for(uint32_t i = 0 ; i<sample;i++){
        _bpi_write_reg(addr + i,0xc0);//USRP2_WRITE_ONE_BPIFLASH_16X
        _bpi_write_reg(addr + i,data_8[0].data[i]);//USRP2_WRITE_ONE_BPIFLASH_16X
    }
}
void flash_iface::_28F00AP30_lock_otp(){
    _bpi_write_reg(0x89,0xc0);//USRP2_WRITE_ONE_BPIFLASH_16X
    _bpi_write_reg(0x89,otp_section_to_command_mapper(save_sec));//USRP2_WRITE_ONE_BPIFLASH_16X
}

void flash_iface::_28F00AP30_read_UID(){
    _bpi_read_reg(0x81,0x90);//USRP2_READ_ONE_BPIFLASH_16X
    for(uint8_t i=0;i<4;i++)
     out_vec.push_back( _bpi_read_reg(0x81+i,0));//USRP2_READ_ONE_BPIFLASH_16X
}




void flash_iface::_IS25LPxx_read(){
    for(uint32_t i=0;i<data_8.size();i++){
        transact_spif(data_8[i].addr,data_8[i].data,data_8[i].num,USRP2_READ_SPIFLASH);
        for(uint32_t j=0;j<data_8[i].num;j++){
            out_vec.push_back(data_8[i].data[j]);
        }
    }
}
void flash_iface::_IS25LPxx_write(){
    for(it_count=0;it_count<data_8.size();it_count++)
        transact_spif(data_8[it_count].addr,data_8[it_count].data,data_8[it_count].num,USRP2_WRITE_SPIFLASH);
}
void flash_iface::_IS25LPxx_erase(){
    std::vector<uint32_t> temp=divide_number_32(sample,1<<16);
//    transact_spif(addr,data_8[0].data,data_8[0].num,USRP2_ERASE_SPIFLASH); //erase first sector
    for(uint32_t i=0;i<temp.size();i++){
        transact_spif(addr,data_8[0].data,data_8[0].num,USRP2_ERASE_SPIFLASH);
        addr += temp[i];
    }
}
void flash_iface::_IS25LPxx_read_otp(){
    if(addr+sample>255+otp_section_addr_map(save_sec).from){
        throw value_error("evrey section of otp have 255 byte");
    }
    for(uint32_t i=0;i<data_8.size();i++){
        transact_spif(data_8[i].addr,data_8[i].data,data_8[i].num,static_cast<usrp2_spiflash_action_t>((otp_section_to_command_mapper(save_sec))|USRP2_READ_OTP_SPIFLASH));
        for(uint32_t j=0;j<data_8[i].num;j++){
            out_vec.push_back(data_8[i].data[j]);
        }
    }
}
void flash_iface::_IS25LPxx_write_otp(){
    if(addr+sample>255+otp_section_addr_map(save_sec).from){
        throw value_error("evrey section of otp have 255 byte");
    }
    for(uint32_t i=0;i<data_8.size();i++){
        transact_spif(data_8[i].addr,data_8[i].data,data_8[i].num,static_cast<usrp2_spiflash_action_t> (otp_section_to_command_mapper(save_sec)|USRP2_WRITE_OTP_SPIFLASH));
    }
}
void flash_iface::_IS25LPxx_erase_otp(){
    uint8_t null_arr[1];
    transact_spif(addr,null_arr,0,static_cast<usrp2_spiflash_action_t>(otp_section_to_command_mapper(save_sec)| (static_cast<uint8_t>(USRP2_ERASE_OTP_SPIFLASH))));

}
void flash_iface::_IS25LPxx_lock_otp(){
    uint8_t null_arr[1];
        transact_spif(addr,null_arr,0,static_cast<usrp2_spiflash_action_t>((otp_section_to_command_mapper(save_sec)|USRP2_LOCK_OTP)));

}
void flash_iface::_IS25LPxx_chip_erase(){
    uint8_t null_arr[1];
    transact_spif(0,null_arr,0,USRP2_ERASE_FULL);
    boost::this_thread::sleep_for(boost::chrono::seconds(45));
}

void flash_iface::_IS25LPxx_read_UID(){
    uint8_t temp_arr[1];
    for(int i=0;i<16;i++){
        transact_spif(i,temp_arr,1,USRP2_READ_UID);
         out_vec.push_back(temp_arr[0]);
    }
}






void flash_iface::_S29GL01GS_entry_ssr_mode(){
    _bpi_write_reg(0x555,0xaa);
    _bpi_write_reg(0x2AA,0x55);
    _bpi_write_reg(0x555,0x88);
}

void flash_iface::_S29GL01GS_entry_ssr_exit(){
    _bpi_write_reg(0x555,0xAA);
    _bpi_write_reg(0x2AA,0x44);
    _bpi_write_reg(0x555,0x90);
    _bpi_write_reg(0x555,0x0);
    _bpi_write_reg(0x555,0xF0);

}
void flash_iface::_S29GL01GS_erase_write_flash_unlock_sequence(){
    /* Write unlock cycle 1 */
    _bpi_write_reg(0x555,0xAA);
    /* Write unlock cycle 2 */
    _bpi_write_reg(0x2AA,0x55);
}


bool flash_iface::_S29GL01GS_erase_sector_flash()
{
  boost::uint32_t start_sector = (_S29GL01GS_SECTOR_START(addr))>>16;
  boost::uint32_t end_sector = (_S29GL01GS_SECTOR_START(addr+sample))>>16;
  uint32_t i;




  /* Sector erase */
  for (i = start_sector; i <= end_sector; i++)
  {
      _S29GL01GS_erase_write_flash_unlock_sequence();
      _bpi_write_reg(0x555,0x80);
      _S29GL01GS_erase_write_flash_unlock_sequence();
      _bpi_write_reg(_S29GL01GS_SECTOR_ADDRESS(i),0x30);
      if (_S29GL01GS_poll_status(FLASH_OP_ERASE) != Successful)
      {
        _bpi_write_reg(0x555,0xF0); //reset exis ASO
        return false;
      }
  }

  return true;
}

bool flash_iface::_S29GL01GS_write_to_buffer(){
    uint32_t addr_temp = addr;
    flash_vec_t temp = divide_number(sample,256);
    for(uint32_t i=0;i<temp.size();i++){
        _S29GL01GS_erase_write_flash_unlock_sequence();
        _bpi_write_reg(_S29GL01GS_SECTOR_START(addr_temp),0x25);
        _bpi_write_reg(_S29GL01GS_SECTOR_START(addr_temp),temp[i]-1);

        uint32_t start_index = i * (256/MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE);
        uint32_t end_index = ( i * (256/MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE) ) +
                std::ceil( static_cast<float>(temp[i]) / MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE) ;

        for(it_count=start_index;it_count < end_index   ;it_count++){
            _bpi_write_N2BYTE(data_16[it_count].addr,data_16[it_count].data,data_16[it_count].num);
        }
        _bpi_write_reg(addr_temp+temp[i]-1,0x29);
        if (_S29GL01GS_poll_status(FLASH_OP_WRITE_BUFFER) != Successful)
        {
          return false;
        }
        addr_temp += temp[i];
    }
    return true;
}
bool flash_iface::_S29GL01GS_chip_erase(){
    _bpi_write_reg(0x555,0xF0); //reset exis ASO
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    _bpi_write_reg(0x555,0xAA);
    _bpi_write_reg(0x2AA,0x55);
    _bpi_write_reg(0x555,0xAA);
    _bpi_write_reg(0x2AA,0x55);
    _bpi_write_reg(0x555,0x10);
    boost::this_thread::sleep(boost::posix_time::seconds(40));
  return 0;
}
boost::uint16_t flash_iface::_S29GL01GS_read_status_register(){
    _bpi_write_reg(0x555,0x70);
    return _bpi_read_reg(0x0);
}

flash_iface::_S29GL01GS_status_state_t flash_iface::_S29GL01GS_poll_status(_S29GL01GS_operation_mode_t operation_mode)
{
      uint16_t failed_code_1;
      uint32_t timeout;
      uint32_t i = 0;

      switch(operation_mode)
      {
        case FLASH_OP_WRITE_BUFFER:
        case FLASH_OP_PROGRAMMING:
          /* Write */
          failed_code_1 = (1 << 4);
          timeout = 20;
          break;
        case FLASH_OP_ERASE:
          /* Erase */
          failed_code_1 = (1 << 5);
          timeout = 3000;
          break;
        default:
          return Invalid_operation_mode;
      }
      boost::uint16_t status_reg = _S29GL01GS_read_status_register();
      while ((status_reg & (1 << 7)) != (1 << 7)) //busy
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        if(i++ > timeout){
          return (Timeout);
        }
        status_reg = _S29GL01GS_read_status_register();
      }
      if(failed_code_1 != (status_reg & (1<<4)))
        return Successful;
      return Program_operation_failed;



}



boost::uint32_t flash_iface::_S29GL01GS_SECTOR_START(boost::uint32_t address){
    return (address & 0xFFFF0000);
}
boost::uint32_t flash_iface::_S29GL01GS_SECTOR_ADDRESS(boost::uint32_t sector){
    return  ((((sector) << 16) & 0xFFFF0000));
}




void flash_iface::_S29GL01GS_read_uid(){
    _bpi_write_reg(0x88,0xF0);
    _bpi_write_reg(0x555,0xAA);
    _bpi_write_reg(0x2AA,0x55);
    _bpi_write_reg(0x555,0x90);
    _bpi_write_reg(0x55,0x98);
    uint16_t data [32] ;
    _bpi_read_N2BYTE(0x82,data,10); //0x82

    for(uint8_t i=0;i<5;i++){
        out_vec.push_back((data[2*i+1]<<8) | (data[2*i]<<0));
    }
    _bpi_write_reg(0x88,0xF0);
}

void flash_iface::_S29GL01GS_read(){
    _bpi_write_reg(0x555,0xF0); //reset exis ASO

    for(uint32_t i=0;i<data_16.size();i++){
        _bpi_read_N2BYTE(data_16[i].addr,data_16[i].data,data_16[i].num);
        for(uint8_t j=0;j<data_16[i].num;j++)
            out_vec.push_back(data_16[i].data[j]);

    }
}

void flash_iface::_S29GL01GS_read_otp(){
    _S29GL01GS_entry_ssr_mode();
    for(uint32_t i=0;i<data_16.size();i++){
        _bpi_read_N2BYTE(data_16[i].addr,data_16[i].data,data_16[i].num);
        for(uint8_t j=0;j<data_16[i].num;j++)
            out_vec.push_back(data_16[i].data[j]);
    }
    _S29GL01GS_entry_ssr_exit();

}
void flash_iface::_S29GL01GS_write_otp(){
    _S29GL01GS_entry_ssr_mode();
    _S29GL01GS_write_to_buffer();
    _S29GL01GS_entry_ssr_exit();
}
void flash_iface::_S29GL01GS_lock_otp(){
    if(save_sec == flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1){
        _bpi_write_reg(0x555,0xaa);
        _bpi_write_reg(0x2aa,0x55);
        _bpi_write_reg(0x555,0x40);
        _bpi_write_reg(0,0xa0);
        _bpi_write_reg(0,0x0);
        _bpi_write_reg(0,0x90);
        _bpi_write_reg(0,0);
        _bpi_write_reg(0,0xF0);

    } else {
        throw pax::value_error("this sectionis not for this flash");
    }
}








i2c_iface::~i2c_iface(void)
{
    //empty
}

spi_iface::~spi_iface(void)
{
    //empty
}

uart_iface::~uart_iface(void)
{
    //empty
}

spi_config_t::spi_config_t(edge_t edge):
    mosi_edge(edge),
    miso_edge(edge)
{
    /* NOP */
}

void i2c_iface::write_eeprom(
    boost::uint16_t addr,
    boost::uint16_t offset,
    const byte_vector_t &bytes
){
    for (size_t i = 0; i < bytes.size(); i++){
        //write a byte at a time, its easy that way
        byte_vector_t cmd = boost::assign::list_of(offset+i)(bytes[i]);
        this->write_i2c(addr, cmd);
        boost::this_thread::sleep(boost::posix_time::milliseconds(10)); //worst case write
    }
}

byte_vector_t i2c_iface::read_eeprom(
    boost::uint16_t addr,
    boost::uint16_t offset,
    size_t num_bytes
){
    byte_vector_t bytes;
    for (size_t i = 0; i < num_bytes; i++){
        //do a zero byte write to start read cycle
        this->write_i2c(addr, byte_vector_t(1, offset+i));
        bytes.push_back(this->read_i2c(addr, 1).at(0));
    }
    return bytes;
}

struct eeprom16_impl : i2c_iface
{
    eeprom16_impl(i2c_iface* internal)
    {
        _internal = internal;
    }
    i2c_iface* _internal;

    byte_vector_t read_i2c(
        boost::uint16_t addr,
        size_t num_bytes
    ){
        return _internal->read_i2c(addr, num_bytes);
    }

    void write_i2c(
        boost::uint16_t addr,
        const byte_vector_t &bytes
    ){
        return _internal->write_i2c(addr, bytes);
    }

    byte_vector_t read_eeprom(
        boost::uint16_t addr,
        boost::uint16_t offset,
        size_t num_bytes
    ){
        byte_vector_t cmd = boost::assign::list_of(offset >> 8)(offset & 0xff);
        this->write_i2c(addr, cmd);
        return this->read_i2c(addr, num_bytes);
    }

    void write_eeprom(
        boost::uint16_t addr,
        boost::uint16_t offset,
        const byte_vector_t &bytes
    ){
        for (size_t i = 0; i < bytes.size(); i++)
        {
            //write a byte at a time, its easy that way
            boost::uint16_t offset_i = offset+i;
            byte_vector_t cmd = boost::assign::list_of(offset_i >> 8)(offset_i & 0xff)(bytes[i]);
            this->write_i2c(addr, cmd);
            boost::this_thread::sleep(boost::posix_time::milliseconds(10)); //worst case write
        }
    }
};

i2c_iface::sptr i2c_iface::eeprom16(void)
{
    return i2c_iface::sptr(new eeprom16_impl(this));
}

boost::uint32_t spi_iface::read_spi(
    int which_slave,
    const spi_config_t &config,
    boost::uint32_t data,
    size_t num_bits
){
    return transact_spi(
        which_slave, config, data, num_bits, true
    );
}

void spi_iface::write_spi(
    int which_slave,
    const spi_config_t &config,
    boost::uint32_t data,
    size_t num_bits
){
    transact_spi(
        which_slave, config, data, num_bits, false
    );
}

void spi_iface::write_spi(
    int which_slave,
    const spi_config_t &config,
    std::vector<boost::uint32_t> &data,
    size_t num_bits
){
    transact_spi(
        which_slave, config, data, num_bits, false
    );
}
