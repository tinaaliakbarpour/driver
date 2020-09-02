//
// Copyright 2010-2013 Ettus Research LLC
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

#ifndef INCLUDED_PAX_TYPES_SERIAL_HPP
#define INCLUDED_PAX_TYPES_SERIAL_HPP

#include <config.h>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <vector>
#include <exception.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <iostream>


namespace pax{

    typedef std::vector<boost::uint8_t> uint8_vector_t;
    typedef std::vector<boost::uint16_t> uint16_vector_t;
    typedef std::vector<boost::uint32_t> uint32_vector_t;



    typedef std::vector<boost::uint16_t> flash_vec_t;

namespace flash {
typedef enum {
    BPI_28F00AP30=0,
    SPI_IS25LPxxx=1,
    BPI_S29GL01GS=2
}flash_type_t;

typedef enum {
    BPI_FLASH_INTERFACAE_LIMIT=0,
    BPI_FLASH_16x =1
}flash_interface_t;



typedef enum {
    BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_1,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_2,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_3,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_4,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_5,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_6,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_7,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_8,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_9,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_10,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_11,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_12,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_13,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_14,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_15,
    BPI_28F00AP30_8_WORD_OTP_REG_SEC_16,
    IS25LPxx_OTP_REG_0x000_TO_0x0ff,
    IS25LPxx_OTP_REG_0x100_TO_0x1ff,
    IS25LPxx_OTP_REG_0x200_TO_0x2ff,
    IS25LPxx_OTP_REG_0x300_TO_0x3ff
}flash_OTP_section_reg_t;

}

#define MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE 32
#define MAX_NUMBER_OF_BUFFER_IN_SPIF_FRIMWARE 64
#define CFI_WB_BASE 0x0B000
#define CFI_BASE 250

    class PAX_API cfi_iface{
    public:
        void cfi_unlock_and_erase_block(uint32_t addr) {
            cfi_write_32_bits(0x04000000+addr,0x0);
            cfi_write_32_bits(0x08000000+addr,0x0);
            uint32_t temp_data = cfi_read_32_bits(0x0c000004);
            while((temp_data  &0x80) == 0x0)
                temp_data = cfi_read_32_bits(0x0c000004);
        }
        uint16_t cfi_read_device_ident(uint32_t addr) {
            return cfi_read_16_bits(0x0e000000 + (addr << 1));
        }
        void cfi_read_all_device_ident() {
            std::cout << "Reading device identifier information\n";
            uint16_t temp = cfi_read_device_ident( 0x0);
            std::cout << "\t Manufacturer code:" << temp << std::endl;
            temp = cfi_read_device_ident( 0x1);
            std::cout << "\t Device ID code:" << temp << std::endl;
            temp = cfi_read_device_ident( 0x5);
            std::cout << "\t RCR:" << temp << std::endl;
            temp = cfi_read_device_ident( 0x2 + ((128*1024)>>1));
            std::cout << "\t Block 0 locked?:" << temp << std::endl;
        }
        uint16_t cfi_query(uint32_t addr) {
            return cfi_read_16_bits(0x0e010000 + (addr << 1));
        }
        void cfi_queries() {
            std::cout << "Querying CFI device\n";
            uint16_t temp = cfi_query( 0x10);
            std::cout << "\t Query ID string:" << temp << std::endl;
            temp = cfi_query( 0x13);
            std::cout << "\t Vendor command set and control interface ID:" << temp << std::endl;
            temp = cfi_query( 0x27);
            std::cout << "\t size 2^n:" << temp << std::endl;
            temp = cfi_query( 0x28);
            std::cout << "\t device interface code:" << temp << std::endl;
            temp = cfi_query( 0x2c);
            std::cout << "\t number of erase block regions:" << temp << std::endl;

        }
        void cfi_data_write_16bits(uint32_t addr, uint32_t data) {
            cfi_write_16_bits(addr,data);
            uint32_t temp_data = cfi_read_32_bits(0x0c000004);
            while((temp_data  &0x80) == 0x0)
                temp_data = cfi_read_32_bits(0x0c000004);
        }
        void cfi_data_write_32bits(uint32_t addr, uint32_t data) {
            cfi_data_write_16bits( addr, data>>16);
            cfi_data_write_16bits( addr+2, data & 0xffff);
        }
        virtual void cfi_write_32_bits(uint32_t addr,uint32_t data) = 0;
        virtual uint32_t cfi_read_32_bits(uint32_t addr) = 0;
        virtual void cfi_write_16_bits(uint32_t addr,uint32_t data) = 0;
        virtual uint16_t cfi_read_16_bits(uint32_t addr) = 0;
        virtual uint8_t cfi_read_8_bits(uint32_t addr) = 0;
    };

    class PAX_API flash_iface : public cfi_iface{
    public:
        virtual void read_flash(uint32_t _addr,flash_vec_t& out,uint32_t num);
        virtual void write_flash(uint32_t _addr,const flash_vec_t& out);
        virtual void erase_flash(uint32_t _addr,uint32_t num);
        virtual void read_otp_flash(uint32_t _addr,flash_vec_t& out,uint32_t num,flash::flash_OTP_section_reg_t read_section);
        virtual void write_otp_flash(uint32_t _addr,const flash_vec_t& out,flash::flash_OTP_section_reg_t write_section);
        virtual void erase_otp_flash(flash::flash_OTP_section_reg_t erase_section);
        virtual void lock_otp_flash(flash::flash_OTP_section_reg_t section);
        virtual flash_vec_t read_uid_flash();
        virtual boost::uint16_t read_flash(uint32_t _addr);
        virtual void write_flash(uint32_t _addr,uint32_t _data);
        virtual void erase_full_flash();
        virtual float get_write_Percentage_pass();

        void set_flash_type (flash::flash_type_t type);
        flash::flash_type_t get_flash_type ();

        void set_flash_interface (flash::flash_interface_t interface_type);
        flash::flash_interface_t get_flash_interface ();


    private:
        boost::uint32_t it_count=0;
        typedef enum{
            ERASE_CMD=0,
            WRITE_CMD=1,
            READ_CMD=2,
            ERASE_OTP_CMD=3,
            WRITE_OTP_CMD=4,
            READ_OTP_CMD=5,
            LOCK_OTP_CMD=6,
            READ_UID_CMD=7,
            ERASE_FULL_CMD=8,
        }cmd_base_t;
        struct addr_bindery{
            uint32_t from;
            uint32_t to;
            addr_bindery(uint32_t _from,uint32_t _to){from=_from;to=_to;}
        };
        template<typename T,uint32_t number_of_buffer>
        class data_splited{
        public:
          T data[number_of_buffer];
          uint32_t addr;
          uint8_t num;
        };

        typedef enum{
            ERASE=0,
            READ,
            WRITE=2
        }minor_cmd;

        void _init_private_field_8x_data_width(const flash_vec_t& _data,uint32_t num,minor_cmd cmd);
        void _init_private_field_16x_data_width(const flash_vec_t& _data,uint32_t num,minor_cmd cmd);

        void init_private_filed(const flash_vec_t& _data,uint32_t num,minor_cmd cmd);
        flash_vec_t divide_number(uint32_t num,uint32_t divider);
        std::vector<uint32_t> divide_number_32(uint32_t num,uint32_t divider);


        addr_bindery otp_section_addr_map(flash::flash_OTP_section_reg_t in);
        boost::uint32_t otp_section_to_command_mapper(flash::flash_OTP_section_reg_t in);
        void transact(cmd_base_t cmd);

        void erase_cmd();
        void write_cmd();
        void read_cmd();
        void erase_otp_cmd();
        void write_otp_cmd();
        void read_otp_cmd();
        void lock_otp_cmd();
        void read_uid_cmd();
        void erase_full_cmd();

    protected:
        typedef enum {
            USRP2_ERASE_SPIFLASH = 0x01,
            USRP2_WRITE_SPIFLASH = 0x02,
            USRP2_READ_SPIFLASH = 0x03,
            USRP2_ERASE_OTP_SPIFLASH = 0x04,
            USRP2_WRITE_OTP_SPIFLASH = 0x05,
            USRP2_READ_OTP_SPIFLASH = 0x06,
            USRP2_LOCK_OTP = 0x07,
            USRP2_READ_UID = 0x08,
            USRP2_ERASE_FULL = 0x09,
            USRP2_OTP_SECTION_0 = 0x00,
            USRP2_OTP_SECTION_1 = 0x10,
            USRP2_OTP_SECTION_2 = 0x20,
            USRP2_OTP_SECTION_3 = 0x30
        } usrp2_spiflash_action_t;

        typedef enum {
            USRP2_WRITE_ONE_BPIFLASH_16X = 0x01,
            USRP2_READ_ONE_BPIFLASH_16X = 0x02,
            USRP2_WRITE_32_BPIFLASH_16X = 0x03,
        } usrp2_bpiflash_16x_action_old_t;

        typedef enum {
            USRP2_READ_2NBYTE_BPIFLASH_16X = 0x04,
            USRP2_WRITE_2NBYTE_BPIFLASH_16X = 0x05
         } usrp2_bpiflash_16x_action_t;


        virtual void transact_spif(uint32_t addr,uint8_t* data,uint8_t nbyte,usrp2_spiflash_action_t action)=0;
        virtual void transact_bpif(uint32_t addr,uint16_t* data,uint8_t n_2byte,usrp2_bpiflash_16x_action_t action)=0;

        virtual boost::uint16_t transact_bpi(boost::uint32_t addr,boost::uint16_t data,usrp2_bpiflash_16x_action_old_t action) = 0;
        virtual boost::uint16_t transact_bpi_buffer(uint32_t addr, uint16_t* data, usrp2_bpiflash_16x_action_old_t action)=0;


    public:
        virtual void _bpi_write_reg(boost::uint32_t address,boost::uint16_t data) = 0;
        virtual boost::uint32_t _bpi_read_reg(boost::uint32_t address) = 0;
        virtual boost::uint32_t _bpi_read_reg(boost::uint32_t address, boost::uint16_t data) = 0;
        virtual void _bpi_write_N2BYTE(uint32_t addr,uint16_t* data,uint8_t n_2byte) = 0;
        virtual void _bpi_read_N2BYTE(uint32_t addr,uint16_t* data,uint8_t n_2byte) = 0;
        virtual void _bpi_write_buffer(uint32_t addr,uint16_t* data) = 0;


    private:





        void _28F00AP30_read();
        void _28F00AP30_write();
        void _28F00AP30_erase();
        void _28F00AP30_write_otp();
        void _28F00AP30_erase_otp();
        void _28F00AP30_read_otp();

        void _28F00AP30_lock_otp();

        void _28F00AP30_read_UID();


        void _IS25LPxx_read();
        void _IS25LPxx_write();
        void _IS25LPxx_erase();
        void _IS25LPxx_read_otp();
        void _IS25LPxx_write_otp();
        void _IS25LPxx_erase_otp();
        void _IS25LPxx_lock_otp();
        void _IS25LPxx_chip_erase();
        void _IS25LPxx_read_UID();






        typedef enum {
            Successful,
            Timeout,
            Write_Buffer_operation_failed,
            Program_operation_failed,
            Device_Error,
            Suspend_mode,
            Invalid_operation_mode
        }_S29GL01GS_status_state_t;

        typedef enum {
            FLASH_OP_WRITE_BUFFER,
            FLASH_OP_PROGRAMMING,
            FLASH_OP_ERASE
        }_S29GL01GS_operation_mode_t;



        void _S29GL01GS_read();
        void _S29GL01GS_read_otp();
        void _S29GL01GS_write_otp();
        void _S29GL01GS_lock_otp();
        void _S29GL01GS_entry_ssr_mode();
        void _S29GL01GS_entry_ssr_exit();
        bool _S29GL01GS_write_to_buffer();
        _S29GL01GS_status_state_t _S29GL01GS_poll_status(_S29GL01GS_operation_mode_t  operation_mode);
        void _S29GL01GS_read_uid();
        inline boost::uint32_t _S29GL01GS_SECTOR_START(boost::uint32_t address);
        inline boost::uint32_t _S29GL01GS_SECTOR_ADDRESS(boost::uint32_t sector);
        void _S29GL01GS_erase_write_flash_unlock_sequence();
        bool _S29GL01GS_erase_sector_flash();
        bool _S29GL01GS_chip_erase();
        boost::uint16_t _S29GL01GS_read_status_register();



        uint32_t sample;
        uint32_t addr;
        std::vector<data_splited<uint8_t,MAX_NUMBER_OF_BUFFER_IN_SPIF_FRIMWARE> > data_8;
        std::vector<data_splited<uint16_t,MAX_NUMBER_OF_BUFFER_IN_BPI_X16_FRIMWARE> > data_16;
        flash_vec_t out_vec;
        flash::flash_OTP_section_reg_t save_sec;
        flash::flash_type_t flash_type;
        flash::flash_interface_t flash_interface;
    };







    /*!
     * The bpi interface class:
     * provide burn to flash
     */
    class PAX_API bpi_iface {
    public:
        typedef boost::shared_ptr<bpi_iface> sptr;

        //! Create an bpi_iface than can talk to 26 bit addressable NOR Flash
        virtual boost::uint16_t transact_bpi(
            boost::uint32_t addr,
            boost::uint32_t data,
            boost::uint8_t action
        ) = 0;
        virtual boost::uint16_t transact_bpi_buffer(uint32_t addr, uint16_t* data, uint8_t action)=0;

         void bpi_write(boost::uint32_t addr,boost::uint32_t data)
        {
            transact_bpi(addr,data,1);
        }
         void bpi_write_buffer(boost::uint32_t addr,boost::uint16_t* data)
        {
            transact_bpi_buffer(addr,data,3);
        }

         boost::uint32_t bpi_read(boost::uint32_t addr)
        {
        return  transact_bpi(addr,0,2);
    }
    };

    /*!
     * The i2c interface class:
     * Provides i2c and eeprom functionality.
     * A subclass should only have to implement the i2c routines.
     * An eeprom implementation comes for free with the interface.
     *
     * The eeprom routines are implemented on top of i2c.
     * The built in eeprom implementation only does single
     * byte reads and byte writes over the i2c interface,
     * so it should be portable across multiple eeproms.
     * Override the eeprom routines if this is not acceptable.
     */


    /*!
     * Byte vector typedef for passing data in and out of I2C interfaces.
     */
    typedef std::vector<boost::uint8_t> byte_vector_t;

    class PAX_API i2c_iface{
    public:
        typedef boost::shared_ptr<i2c_iface> sptr;

        virtual ~i2c_iface(void);

        //! Create an i2c_iface than can talk to 16 bit addressable EEPROMS
        i2c_iface::sptr eeprom16(void);

        /*!
         * Write bytes over the i2c.
         * \param addr the address
         * \param buf the vector of bytes
         */
        virtual void write_i2c(
            boost::uint16_t addr,
            const byte_vector_t &buf
        ) = 0;

        /*!
         * Read bytes over the i2c.
         * \param addr the address
         * \param num_bytes number of bytes to read
         * \return a vector of bytes
         */
        virtual byte_vector_t read_i2c(
            boost::uint16_t addr,
            size_t num_bytes
        ) = 0;

        /*!
         * Write bytes to an eeprom.
         * \param addr the address
         * \param offset byte offset
         * \param buf the vector of bytes
         */
        virtual void write_eeprom(
            boost::uint16_t addr,
            boost::uint16_t offset,
            const byte_vector_t &buf
        );

        /*!
         * Read bytes from an eeprom.
         * \param addr the address
         * \param offset byte offset
         * \param num_bytes number of bytes to read
         * \return a vector of bytes
         */
        virtual byte_vector_t read_eeprom(
            boost::uint16_t addr,
            boost::uint16_t offset,
            size_t num_bytes
        );
    };

    /*!
     * The SPI configuration struct:
     * Used to configure a SPI transaction interface.
     */
    struct PAX_API spi_config_t{
        /*!
         * The edge type specifies when data is valid
         * relative to the edge of the serial clock.
         */
        enum edge_t{
            EDGE_RISE = 'r',
            EDGE_FALL = 'f'
        };

        //! on what edge is the mosi data valid?
        edge_t mosi_edge;

        //! on what edge is the miso data valid?
        edge_t miso_edge;

        /*!
         * Create a new spi config.
         * \param edge the default edge for mosi and miso
         */
        spi_config_t(edge_t edge = EDGE_RISE);
    };

    /*!
     * The SPI interface class.
     * Provides routines to transact SPI and do other useful things which haven't been defined yet.
     */
    class PAX_API spi_iface{
    public:
        typedef boost::shared_ptr<spi_iface> sptr;

        virtual ~spi_iface(void);

        /*!
        * Perform a spi transaction.
        * \param which_slave the slave device number
        * \param config spi config args
        * \param data the bits to write
        * \param num_bits how many bits in data
        * \param readback true to readback a value
        * \return spi data if readback set
        */
        virtual boost::uint32_t transact_spi(
            int which_slave,
            const spi_config_t &config,
            boost::uint32_t data,
            size_t num_bits,
            bool readback
        ) = 0;

        /*!
        * Perform multiple spi transactions.
        * \param which_slave the slave device number
        * \param config spi config args
        * \param data vector of spi words to write
        * \param num_bits how many bits in each data word
        * \param readback true to readback a value
        * \return spi data if readback set (SBM: just the first one for now)
        */
        virtual boost::uint32_t transact_spi(
            int which_slave,
            const spi_config_t &config,
            std::vector<boost::uint32_t> &data,
            size_t num_bits,
            bool readback
        ) = 0;

        /*!
        * Read from the SPI bus.
        * \param which_slave the slave device number
        * \param config spi config args
        * \param data the bits to write out (be sure to set write bit)
        * \param num_bits how many bits in data
        * \return spi data
        */
        virtual boost::uint32_t read_spi(
            int which_slave,
            const spi_config_t &config,
            boost::uint32_t data,
            size_t num_bits
        );

        /*!
        * Write to the SPI bus.
        * \param which_slave the slave device number
        * \param config spi config args
        * \param data the bits to write
        * \param num_bits how many bits in data
        */
        virtual void write_spi(
            int which_slave,
            const spi_config_t &config,
            boost::uint32_t data,
            size_t num_bits
        );

        /*!
        * Write multiple data to the SPI bus.
        * \param which_slave the slave device number
        * \param config spi config args
        * \param data vector of spi words to write
        * \param num_bits how many bits in each data (must be the same for all)
        */
        virtual void write_spi(
            int which_slave,
            const spi_config_t &config,
            std::vector<uint32_t> &data,
            size_t num_bits
        );
    };

    /*!
     * UART interface to write and read bytes.
     */
    class PAX_API uart_iface{
    public:
        typedef boost::shared_ptr<uart_iface> sptr;

        virtual ~uart_iface(void);

        /*!
         * Write to a serial port.
         * \param buf the data to write
         */
        virtual void write_uart(const std::string &buf) = 0;

        /*!
         * Read from a serial port.
         * Reads until complete line or timeout.
         * \param timeout the timeout in seconds
         * \return the data read from the serial port
         */
        virtual std::string read_uart(double timeout) = 0;
    };

  } //namespace pax

#endif /* INCLUDED_PAX_TYPES_SERIAL_HPP */
