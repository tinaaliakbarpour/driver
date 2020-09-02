//
// Copyright 2010-2012,2014 Ettus Research LLC
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

//#include "usrp2_regs.hpp"
//#include "usrp2_impl.hpp"
#include <device_addr.hpp>

#include <fw_common.h>
#include <pax_iface.h>
#include <exception.hpp>
#include <dict.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp> //used for htonl and ntohl
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/functional/hash.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iostream>
#include <platform.hpp>
#include <tasks.hpp>
#include <pax_regs.hpp>
#include <safe_call.h>

namespace fs = boost::filesystem;

static const double CTRL_RECV_TIMEOUT = 1.0;
static const size_t CTRL_RECV_RETRIES = 3;

//custom timeout error for retry logic to catch/retry
struct timeout_error : pax::runtime_error
{
    timeout_error(const std::string &what):
        pax::runtime_error(what)
    {
        //NOP
    }
};

static const boost::uint32_t MIN_PROTO_COMPAT_SPI = 7;
static const boost::uint32_t MIN_PROTO_COMPAT_I2C = 7;
// The register compat number must reflect the protocol compatibility
// and the compatibility of the register mapping (more likely to change).
static const boost::uint32_t MIN_PROTO_COMPAT_REG = 10;
static const boost::uint32_t MIN_PROTO_COMPAT_UART = 7;

class pax_iface_impl : public pax_iface{
public:
/***********************************************************************
 * Structors
 **********************************************************************/
    pax_iface_impl(pax::transport::udp_simple::sptr ctrl_transport):
        _ctrl_transport(ctrl_transport),
        _ctrl_seq_num(0),
        _protocol_compat(USRP2_FW_COMPAT_NUM) //initialized below...
    {

        //Obtain the firmware's compat number.
        //Save the response compat number for communication.
        //TODO can choose to reject certain older compat numbers
        usrp2_ctrl_data_t ctrl_data = usrp2_ctrl_data_t();

        ctrl_data.id = htonl(USRP2_CTRL_ID_WAZZUP_BRO);
        ctrl_data = ctrl_send_and_recv(ctrl_data, 0, ~0);
        if (ntohl(ctrl_data.id) != USRP2_CTRL_ID_WAZZUP_DUDE)
            throw pax::runtime_error("firmware not responding");
        _protocol_compat = ntohl(ctrl_data.proto_ver);

    //    mb_eeprom = mboard_eeprom_t(*this, "pax8");
    }

    ~pax_iface_impl(void){PAX_SAFE_CALL(
        this->lock_device(false);
    )}

/***********************************************************************
 * Device locking
 **********************************************************************/

    void lock_device(bool lock){
        if (lock){
            this->pokefw(U2_FW_REG_LOCK_GPID, pax::get_process_hash());
            _lock_task = pax::task::make(boost::bind(&pax_iface_impl::lock_task, this));
        }
        else{
            _lock_task.reset(); //shutdown the task
            this->pokefw(U2_FW_REG_LOCK_TIME, 0); //unlock
        }
    }

    bool is_device_locked(void){
        //never assume lock with fpga image mismatch
        if ((this->peek32(U2_REG_COMPAT_NUM_RB) >> 16) != USRP2_FPGA_COMPAT_NUM) return false;

        boost::uint32_t lock_time = this->peekfw(U2_FW_REG_LOCK_TIME);
        boost::uint32_t lock_gpid = this->peekfw(U2_FW_REG_LOCK_GPID);

        //may not be the right tick rate, but this is ok for locking purposes
        const boost::uint32_t lock_timeout_time = boost::uint32_t(3*DSP_CLOCK_RATE);

        //if the difference is larger, assume not locked anymore
        if ((lock_time & 1) == 0) return false; //bit0 says unlocked
        const boost::uint32_t time_diff = this->get_curr_time() - lock_time;
        if (time_diff >= lock_timeout_time) return false;

        //otherwise only lock if the device hash is different that ours
        return lock_gpid != pax::get_process_hash();
    }

    void lock_task(void){
        //re-lock in task
        this->pokefw(U2_FW_REG_LOCK_TIME, this->get_curr_time());
        //sleep for a bit
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    }

    boost::uint32_t get_curr_time(void){
        return this->peek32(U2_REG_TIME64_LO_RB_IMM) | 1; //bit 1 says locked
    }

/***********************************************************************
 * Peek and Poke
 **********************************************************************/



    void poke32(wb_addr_type addr, boost::uint32_t data){
        this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FPGA_POKE32>(addr, data);
    }

    boost::uint32_t  peek32(wb_addr_type addr){
        return this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FPGA_PEEK32>(addr);
    }

    void poke16(wb_addr_type addr, boost::uint16_t data){
        this->get_reg<boost::uint16_t, USRP2_REG_ACTION_FPGA_POKE16>(addr, data);
    }

    boost::uint16_t peek16(wb_addr_type addr){
        return this->get_reg<boost::uint16_t, USRP2_REG_ACTION_FPGA_PEEK16>(addr);
    }

    void pokefw(wb_addr_type addr, boost::uint32_t data)
    {
        this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_POKE32>(addr, data);
    }

    boost::uint32_t peekfw(wb_addr_type addr)
    {
        return this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_PEEK32>(addr);
    }



    template <class T, usrp2_reg_action_t action>
    T get_reg(wb_addr_type addr, T data = 0){
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_GET_THIS_REGISTER_FOR_ME_BRO);
        out_data.data.reg_args.addr = htonl(addr);
        out_data.data.reg_args.data = htonl(boost::uint32_t(data));
        out_data.data.reg_args.action = action;

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_REG);
        PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_OMG_GOT_REGISTER_SO_BAD_DUDE);
        return T(ntohl(in_data.data.reg_args.data));
    }

  /***********************************************************************
    * BPI
  **********************************************************************/
    boost::uint16_t transact_bpi(uint32_t addr, uint16_t data, usrp2_bpiflash_16x_action_old_t action)
    { 
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(PAX_CTRL_ID_TRANSACT_BPI_BRO);
        out_data.data.bpi_16x_args.addr=htonl(addr);
        out_data.data.bpi_16x_args.data[0]=(htons(data));
        out_data.data.bpi_16x_args.action=static_cast<boost::uint8_t>(action);
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        PAX_ASSERT_THROW(ntohl(in_data.id) == PAX_CTRL_ID_TRANSACTED_BPI_DUDE);
        return (ntohs(in_data.data.bpi_16x_args.data[0]));
    }

    boost::uint16_t transact_bpi_buffer(uint32_t addr, uint16_t* data, usrp2_bpiflash_16x_action_old_t action)
    {
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(PAX_CTRL_ID_TRANSACT_BPI_BRO);
        out_data.data.bpi_16x_args.addr=htonl(addr);
        for(uint8_t i=0;i<32;i++)
            out_data.data.bpi_16x_args.data[i] = htons(data[i]);
        //memcpy(out_data.data.bpi_16x_args.data,data,64);
        out_data.data.bpi_16x_args.action=static_cast<boost::uint8_t>(action);
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        PAX_ASSERT_THROW(ntohl(in_data.id) == PAX_CTRL_ID_TRANSACTED_BPI_DUDE);
        return ntohs(in_data.data.bpi_16x_args.data[0]);
    }
    void transact_bpif( uint32_t addr, uint16_t* data,  uint8_t n_2byte, usrp2_bpiflash_16x_action_t action ){
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(PAX_CTRL_ID_TRANSACT_BPI_BRO);
        out_data.data.bpi_16x_args.addr=htonl(addr);

        for(uint8_t i=0;i<n_2byte;i++){
            out_data.data.bpi_16x_args.data[i]=(htons(data[i]));
        }
        out_data.data.bpi_16x_args.action=static_cast<boost::uint8_t>(action);
        out_data.data.bpi_16x_args.n_2byte=n_2byte;

        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        PAX_ASSERT_THROW(ntohl(in_data.id) == PAX_CTRL_ID_TRANSACTED_BPI_DUDE);
        if(action == USRP2_READ_2NBYTE_BPIFLASH_16X){
            for(uint8_t i=0;i<in_data.data.bpi_16x_args.n_2byte;i++){
                data[i]=htons(in_data.data.bpi_16x_args.data[i]);
            }
        }
      }

    /***********************************************************************
      * spif
    **********************************************************************/
      void transact_spif(uint32_t addr, uint8_t* data,uint8_t nbyte,pax::flash_iface::usrp2_spiflash_action_t action)
      {
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(PAX_CTRL_ID_TRANSACT_SPIF_BRO);
        out_data.data.spif_args.addr=htonl(addr);
        for(int i=0;i<nbyte;i++){
          out_data.data.spif_args.data[i]=(data[i]);
        }
        out_data.data.spif_args.action= static_cast<boost::uint8_t>(action);
        out_data.data.spif_args.nbyte=nbyte;

        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        PAX_ASSERT_THROW(ntohl(in_data.id) == PAX_CTRL_ID_TRANSACTED_SPIF_DUDE);
        for(uint8_t i=0;i<nbyte;i++){
          data[i]=in_data.data.spif_args.data[i];
        }
      }


/***********************************************************************
 * SPI
 **********************************************************************/
    boost::uint32_t transact_spi(
        int which_slave,
        const pax::spi_config_t &config,
        boost::uint32_t data,
        size_t num_bits,
        bool readback
    ){

        static const pax::dict<pax::spi_config_t::edge_t, int> spi_edge_to_otw = boost::assign::map_list_of
            (pax::spi_config_t::EDGE_RISE, USRP2_CLK_EDGE_RISE)
            (pax::spi_config_t::EDGE_FALL, USRP2_CLK_EDGE_FALL)
        ;

        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO);
        out_data.data.spi_args.dev = htonl(which_slave);
        out_data.data.spi_args.miso_edge = spi_edge_to_otw[config.miso_edge];
        out_data.data.spi_args.mosi_edge = spi_edge_to_otw[config.mosi_edge];
        out_data.data.spi_args.readback = (readback)? 1 : 0;
        out_data.data.spi_args.num_bits = num_bits;
        out_data.data.spi_args.data[0] = htonl(data);
        //SBM, default number of data words
        out_data.data.spi_args.num_data = 1;

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE);

        return ntohl(in_data.data.spi_args.data[0]);
    }

    // SBM, transact multiple spi (15 max)
    boost::uint32_t transact_spi(
        int which_slave,
        const pax::spi_config_t &config,
        std::vector<boost::uint32_t> &data,
        size_t num_bits,
        bool readback
    ){
        PAX_ASSERT_THROW(data.size()<16);
        static const pax::dict<pax::spi_config_t::edge_t, int> spi_edge_to_otw = boost::assign::map_list_of
            (pax::spi_config_t::EDGE_RISE, USRP2_CLK_EDGE_RISE)
            (pax::spi_config_t::EDGE_FALL, USRP2_CLK_EDGE_FALL)
        ;

        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO);
        out_data.data.spi_args.dev = htonl(which_slave);
        out_data.data.spi_args.miso_edge = spi_edge_to_otw[config.miso_edge];
        out_data.data.spi_args.mosi_edge = spi_edge_to_otw[config.mosi_edge];
        out_data.data.spi_args.readback = (readback)? 1 : 0;
        out_data.data.spi_args.num_bits = num_bits;
        for (size_t i=0; i<data.size(); i++)
            out_data.data.spi_args.data[i] = htonl(data[i]);
        out_data.data.spi_args.num_data = data.size();

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE);

        // all results are returned but this function is used for spi_write only (not needed)
        return ntohl(in_data.data.spi_args.data[0]);
    }

/***********************************************************************
 * I2C
 **********************************************************************/
    void write_i2c(boost::uint16_t addr, const pax::byte_vector_t &buf){
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_WRITE_THESE_I2C_VALUES_BRO);
        out_data.data.i2c_args.addr = addr;
        out_data.data.i2c_args.bytes = buf.size();

        //limitation of i2c transaction size
        PAX_ASSERT_THROW(buf.size() <= sizeof(out_data.data.i2c_args.data));

        //copy in the data
        std::copy(buf.begin(), buf.end(), out_data.data.i2c_args.data);

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_I2C);
        PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_COOL_IM_DONE_I2C_WRITE_DUDE);
    }

    pax::byte_vector_t read_i2c(boost::uint16_t addr, size_t num_bytes){
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_DO_AN_I2C_READ_FOR_ME_BRO);
        out_data.data.i2c_args.addr = addr;
        out_data.data.i2c_args.bytes = num_bytes;

        //limitation of i2c transaction size
        PAX_ASSERT_THROW(num_bytes <= sizeof(out_data.data.i2c_args.data));

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_I2C);
        PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_HERES_THE_I2C_DATA_DUDE);
        PAX_ASSERT_THROW(in_data.data.i2c_args.bytes == num_bytes);

        //copy out the data
        pax::byte_vector_t result(num_bytes);
        std::copy(in_data.data.i2c_args.data, in_data.data.i2c_args.data + num_bytes, result.begin());
        return result;
    }

    /***********************************************************************
     * UART
     **********************************************************************/
        void write_uart(boost::uint16_t addr, const pax::byte_vector_t &buf){
            //setup the out data
            usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
            out_data.id = htonl(USRP2_CTRL_ID_HEY_WRITE_THIS_UART_FOR_ME_BRO);
            out_data.data.uart_args.dev = addr;
            out_data.data.uart_args.bytes = buf.size();

            //limitation of i2c transaction size
            PAX_ASSERT_THROW(buf.size() <= sizeof(out_data.data.uart_args.data));

            //copy in the data
           // std::copy(buf.begin(), buf.end(), out_data.data.uart_args.data);
            for(uint32_t i=0;i<buf.size();i++){
                out_data.data.uart_args.data[i]=buf[i];
            }
            //send and recv
            usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_UART);
            PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_MAN_I_TOTALLY_WROTE_THAT_UART_DUDE);
        }

        pax::byte_vector_t read_uart(boost::uint16_t addr, size_t num_bytes){
            //setup the out data
            usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
            out_data.id = htonl(USRP2_CTRL_ID_SO_LIKE_CAN_YOU_READ_THIS_UART_BRO);
            out_data.data.uart_args.dev = addr;
            out_data.data.uart_args.bytes = num_bytes;

            //limitation of i2c transaction size
            PAX_ASSERT_THROW(num_bytes <= sizeof(out_data.data.uart_args.data));

            //send and recv
            usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_UART);
            PAX_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_I_HELLA_READ_THAT_UART_DUDE);
            //PAX_ASSERT_THROW(in_data.data.uart_args.bytes == num_bytes);

            //copy out the data
            pax::byte_vector_t result(in_data.data.uart_args.bytes);
            std::copy(in_data.data.uart_args.data, in_data.data.uart_args.data + in_data.data.uart_args.bytes, result.begin());
            return result;
        }

/***********************************************************************
 * Send/Recv over control
 **********************************************************************/
    usrp2_ctrl_data_t ctrl_send_and_recv(
        const usrp2_ctrl_data_t &out_data,
        boost::uint32_t lo = USRP2_FW_COMPAT_NUM,
        boost::uint32_t hi = USRP2_FW_COMPAT_NUM
    ){
        boost::mutex::scoped_lock lock(_ctrl_mutex);

        for (size_t i = 0; i < CTRL_RECV_RETRIES; i++){
            try{
                return ctrl_send_and_recv_internal(out_data, lo, hi, CTRL_RECV_TIMEOUT/CTRL_RECV_RETRIES);
            }
            catch(const timeout_error &e){
              std::cout
                    << "Control packet attempt " << i
                    << ", sequence number " << _ctrl_seq_num
                    << ":\n" << e.what() << std::endl;
            }
        }
        throw pax::runtime_error("link dead: timeout waiting for control packet ACK");
    }

    usrp2_ctrl_data_t ctrl_send_and_recv_internal(
        const usrp2_ctrl_data_t &out_data,
        boost::uint32_t lo, boost::uint32_t hi,
        const double timeout
    ){
        //fill in the seq number and send
        usrp2_ctrl_data_t out_copy = out_data;
        out_copy.proto_ver = htonl(_protocol_compat);
        out_copy.seq = htonl(++_ctrl_seq_num);
        _ctrl_transport->send(boost::asio::buffer(&out_copy, sizeof(usrp2_ctrl_data_t)));

        //loop until we get the packet or timeout
        boost::uint8_t usrp2_ctrl_data_in_mem[pax::transport::udp_simple::mtu]; //allocate max bytes for recv
        const usrp2_ctrl_data_t *ctrl_data_in = reinterpret_cast<const usrp2_ctrl_data_t *>(usrp2_ctrl_data_in_mem);
        while(true){
            size_t len = _ctrl_transport->recv(boost::asio::buffer(usrp2_ctrl_data_in_mem), timeout);
            boost::uint32_t compat = ntohl(ctrl_data_in->proto_ver);
            if(len >= sizeof(boost::uint32_t) and (hi < compat or lo > compat)){
                throw pax::runtime_error(str(boost::format(
                    "\nPlease update the firmware and FPGA images for your device.\n"
                    "See the application notes for PAX8-Series for instructions.\n"
                    "Expected protocol compatibility number %s, but got %d:\n"
                    "The firmware build is not compatible with the host code build.\n"
                    "%s\n"
                ) % ((lo == hi)? (boost::format("%d") % hi) : (boost::format("[%d to %d]") % lo % hi))
                  % compat % this->images_warn_help_message()));
            }
            if (len >= sizeof(usrp2_ctrl_data_t) and ntohl(ctrl_data_in->seq) == _ctrl_seq_num){
                return *ctrl_data_in;
            }
            if (len == 0) break; //timeout
            //didnt get seq or bad packet, continue looking...
        }
        throw timeout_error("no control response, possible packet loss");
    }

    rev_type get_rev(void){
        std::string hw = "";//mb_eeprom["hardware"];
        if (hw.empty()) return USRP_NXXX;
        switch (boost::lexical_cast<boost::uint16_t>(hw)){
        case 0x0300:
        case 0x0301: return USRP2_REV3;
        case 0x0400: return USRP2_REV4;
        case 0x0A00: return USRP_N200;
        case 0x0A01: return USRP_N210;
        case 0x0A10: return USRP_N200_R4;
        case 0x0A11: return USRP_N210_R4;
        }
        return USRP_NXXX; //unknown type
    }

    const std::string get_cname(void){
        switch(this->get_rev()){
        case USRP2_REV3: return "USRP2 r3";
        case USRP2_REV4: return "USRP2 r4";
        case USRP_N200: return "N200";
        case USRP_N210: return "N210";
        case USRP_N200_R4: return "N200r4";
        case USRP_N210_R4: return "N210r4";
        case USRP_NXXX: return "N???";
        }
        PAX_THROW_INVALID_CODE_PATH();
    }

    const std::string get_fw_version_string(void){
        boost::uint32_t minor = this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_PEEK32>(U2_FW_REG_VER_MINOR);
        return str(boost::format("%u.%u") % _protocol_compat % minor);
    }

    std::string images_warn_help_message(void){
        //determine the images names
        std::string fw_image, fpga_image;
        switch(this->get_rev()){
        case USRP2_REV3:   fpga_image = "usrp2_fpga.bin";        fw_image = "usrp2_fw.bin";     break;
        case USRP2_REV4:   fpga_image = "usrp2_fpga.bin";        fw_image = "usrp2_fw.bin";     break;
        case USRP_N200:    fpga_image = "usrp_n200_r2_fpga.bin"; fw_image = "usrp_n200_fw.bin"; break;
        case USRP_N210:    fpga_image = "usrp_n210_r2_fpga.bin"; fw_image = "usrp_n210_fw.bin"; break;
        case USRP_N200_R4: fpga_image = "usrp_n200_r4_fpga.bin"; fw_image = "usrp_n200_fw.bin"; break;
        case USRP_N210_R4: fpga_image = "usrp_n210_r4_fpga.bin"; fw_image = "usrp_n210_fw.bin"; break;
        default: break;
        }
        if (fw_image.empty() or fpga_image.empty()) return "";

        //does your platform use sudo?
        std::string sudo;
        #if defined(PAX_PLATFORM_LINUX) || defined(PAX_PLATFORM_MACOS)
            sudo = "sudo ";
        #endif


        //look up the real FS path to the images
        std::string fw_image_path, fpga_image_path;
        try{
            fw_image_path = "";//pax::find_image_path(fw_image);
            fpga_image_path = "";//pax::find_image_path(fpga_image);
        }
        catch(const std::exception &){
            return str(boost::format("Call AmirRahmat"));
        }

        //escape char for multi-line cmd + newline + indent?
        #ifdef PAX_PLATFORM_WIN32
            const std::string ml = "^\n    ";
        #else
            const std::string ml = "\\\n    ";
        #endif

        //create the burner commands
        if (this->get_rev() == USRP2_REV3 or this->get_rev() == USRP2_REV4){
            const std::string card_burner = "";//(fs::path(pax::get_pkg_path()) / PAX_LIB_DIR / "pax" / "utils" / "usrp2_card_burner.py").string();
            const std::string card_burner_cmd = str(boost::format("\"%s%s\" %s--fpga=\"%s\" %s--fw=\"%s\"") % sudo % card_burner % ml % fpga_image_path % ml % fw_image_path);
            return ("Call AmirRahmat!!");//boost::format("%s\n%s") % print_utility_error("pax_images_downloader.py") % card_burner_cmd);
        }
        else{
            const std::string addr = _ctrl_transport->get_recv_addr();
            const std::string net_burner_path ="";// (fs::path(pax::get_pkg_path()) / PAX_LIB_DIR / "pax" / "utils" / "usrp_n2xx_simple_net_burner").string();
            const std::string net_burner_cmd = "";//str(boost::format("\"%s\" %s--addr=\"%s\"") % net_burner_path % ml % addr);
            return str(boost::format("%s\n%s") % ("Call AmirRahmat") );
        }
    }



    /***********************************************************************
     * CFI
     **********************************************************************/

    void set_fifo_ctrl(boost::shared_ptr<usrp2_fifo_ctrl> fifo_ctrl){_fifo_ctrl = fifo_ctrl;}

    void cfi_write_32_bits(uint32_t addr,uint32_t data) {
        addr = (addr & 0x1fffffff) | ( (((uint32_t)0) <<29));
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * CFI_BASE, data);
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * (CFI_BASE + 1), addr);
        while(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 3) == 0);
    }
    uint32_t cfi_read_32_bits(uint32_t addr) {
        addr = (addr & 0x1fffffff) | ( (((uint32_t)1) <<29));
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * (CFI_BASE + 1), addr);
        while(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 3) == 0);
        return (_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 2));

    }
    void cfi_write_16_bits(uint32_t addr,uint32_t data) {
        uint32_t temp = (data & 0xffff);
        data =  temp | (temp << 16);
        addr = (addr & 0x1fffffff) | ( (((uint32_t)2) <<29));
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * CFI_BASE, data);
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * (CFI_BASE + 1), addr);
        while(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 3) == 0);

    }
    uint16_t cfi_read_16_bits(uint32_t addr) {
        addr = (addr & 0x1fffffff) | ( (((uint32_t)3) <<29));
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * (CFI_BASE + 1), addr);
        while(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 3) == 0);
        return (uint16_t)(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 2));
    }
    uint8_t cfi_read_8_bits(uint32_t addr) {
        addr = (addr & 0x1fffffff) | ( (((uint32_t)4) <<29));
        _fifo_ctrl->poke32(SETTING_REGS_BASE + 4 * (CFI_BASE + 1), addr);
        while(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 3) == 0);
        return (uint8_t)(_fifo_ctrl->peek32(SETTING_REGS_BASE + 4 * 2));
    }
    inline void _bpi_write_reg(boost::uint32_t address,boost::uint16_t data){
        #ifdef __CFI__
            cfi_data_write_16bits(address, data);
        #else
            transact_bpi(address,data,USRP2_WRITE_ONE_BPIFLASH_16X);
        #endif
    }
    inline boost::uint32_t _bpi_read_reg(boost::uint32_t address){
        #ifdef __CFI__
            return cfi_read_16_bits(address);
        #else
            return transact_bpi(address,address,USRP2_READ_ONE_BPIFLASH_16X);
        #endif
    }

    inline boost::uint32_t _bpi_read_reg(boost::uint32_t address,boost::uint16_t data){
        #ifdef __CFI__
            return cfi_read_16_bits(address);
        #else
            return transact_bpi(address,data,USRP2_READ_ONE_BPIFLASH_16X);
        #endif
    }

    inline void _bpi_write_N2BYTE(uint32_t addr,uint16_t* data,uint8_t n_2byte){
        #ifdef __CFI__
            for(uint8_t i = 0; i < n_2byte; i++)
                cfi_data_write_16bits(addr + 2 * i,*(data + i));
        #else
            transact_bpif(addr, data, n_2byte, USRP2_WRITE_2NBYTE_BPIFLASH_16X);
        #endif
    }
    inline void _bpi_read_N2BYTE(uint32_t addr,uint16_t* data,uint8_t n_2byte){
        #ifdef __CFI__
            for(uint8_t i = 0; i < n_2byte; i++)
                *(data + i) = cfi_read_16_bits(addr);
        #else
            transact_bpif(addr, data, n_2byte, USRP2_READ_2NBYTE_BPIFLASH_16X);
        #endif
    }
    inline void _bpi_write_buffer(uint32_t addr,uint16_t* data){
        #ifdef __CFI__
            for(uint8_t i = 0; i < 32; i++)
                cfi_data_write_16bits(addr + 2 * i,*(data + i));
        #else
            transact_bpi_buffer(addr, data, USRP2_WRITE_32_BPIFLASH_16X);
        #endif
    }


private:
    //this lovely lady makes it all possible
    pax::transport::udp_simple::sptr _ctrl_transport;

    //used in send/recv
    boost::mutex _ctrl_mutex;
    boost::uint32_t _ctrl_seq_num;
    boost::uint32_t _protocol_compat;

    //lock thread stuff
    pax::task::sptr _lock_task;
    boost::shared_ptr<usrp2_fifo_ctrl> _fifo_ctrl;
};

/***********************************************************************
 * Public make function for usrp2 interface
 **********************************************************************/

pax_iface::sptr pax_iface::make(pax::transport::udp_simple::sptr ctrl_transport){
   return pax_iface::sptr(new pax_iface_impl(ctrl_transport));
}


