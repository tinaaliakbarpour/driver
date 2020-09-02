#include "micron_impl.hpp"

    pax::micron_impl::micron_impl(pax_iface::sptr iface):iface(iface){}
    bool pax::micron_impl::micron_block_unlock(uint32_t block_addr)
    {
        uint8_t k=0;
        while(true)
        {
            k++;
            iface->write_flash(block_addr, 0x60);   // lock setup
            iface->write_flash(block_addr, 0xD0);   // unlock confirm
            iface->write_flash(block_addr,0x90);  // read device register
            boost::uint32_t reg=iface->read_flash(block_addr+2);
            if((reg&0x01)==0)
            {
                iface->write_flash(0,0xFF);
                return true;
            }
            else if((reg & 0x02)==2)
            {
                throw pax::runtime_error("Device in lock down!\n");
                return false;
            }

            if(k>10)
                break;
        }
        return false;
    }

    bool pax::micron_impl::micron_blank_check(boost::uint32_t block_addr)
    {
        iface->write_flash(block_addr, 0x50);   // clear status register
        iface->write_flash(block_addr,0x0BC);
        iface->write_flash(block_addr,0x0D0);
        boost::uint32_t blank_check=0;
        boost::uint32_t status ;
        while (!blank_check){
            status = iface->read_flash(0); // does address matter????
            blank_check=!!(status&0x80);  // check ready status
        }
        if((status|(1<<5 | 1<<4))==0)
        {
            iface->write_flash(block_addr, 0x50);   // clear status register
            iface->write_flash(0,0xFF);
            return true;
        }
        iface->write_flash(block_addr, 0x50);   // clear status register
        iface->write_flash(0,0xFF);
        return false;

    }

    bool pax::micron_impl::micron_erase(boost::uint32_t block_num)
    {
        uint32_t block_addr;
        if(block_num<1023)
            block_addr=block_num*0x10000;
        else if(block_num<1027)
            block_addr=(1022*0x10000)+(block_num-1022)*0x4000;
        else
            throw pax::runtime_error("Block number not in range!");

        iface->write_flash(block_addr, 0x50);   // clear status register

        micron_block_unlock(block_addr);
        if(micron_blank_check(block_addr))
            return true;

        // next, erase the block
        iface->write_flash(block_addr, 0x20); // block erase
        iface->write_flash(block_addr, 0xD0);   // erase confirm
        bool erase_complete=false;
        int status;
        std::cout << "Erase started." << std::endl;
        while (!erase_complete){
            status = iface->read_flash(0); // does address matter????
            erase_complete=!!(status&0x80);  // check ready status
        }
        iface->write_flash(block_addr, 0x50);   // clear status register
        iface->write_flash(block_addr,0x70);
        status = iface->read_flash(block_addr);
        if(status&(1<<1))
            throw pax::runtime_error("block is locked!!!!");
        iface->write_flash(0,0xFF);
        std::cout << "Erase finished." << std::endl;
        return true;
    }

    bool pax::micron_impl::micron_write(boost::uint32_t block_num,std::vector<boost::uint16_t> &data)
    {
        micron_erase(block_num);
        uint32_t block_addr;
        if(block_num<1023)
            block_addr=block_num*0x10000;
        else if(block_num<1027)
            block_addr=(1022*0x10000)+(block_num-1022)*0x4000;
        else
            throw pax::runtime_error("Block number not in range!");

        boost::uint32_t offset=0;
        boost::uint32_t block_size=block_num<1023?0x10000:0x4000;
        assert(block_size>=data.size());
        while(data.size()>offset)
        {
            iface->write_flash(block_addr,0xE8);
            boost::uint32_t command_complete=0;
            boost::uint32_t  status;

            while (!command_complete){
                status = iface->read_flash(0); // does address matter????
                command_complete=!!(status&0x80);  // check ready status
            }

            boost::uint32_t n_word=std::min((size_t)512,(data.size()-offset));
            iface->write_flash(block_addr,n_word-1);
            for(boost::uint32_t i=0;i<(n_word);i+=32)
            {
                if((data.size()-offset)<32)
                    for(size_t jj=0;jj<(data.size()-offset);jj++)
                    {
//                        boost::uint16_t temp=pax::byteswap(data[i+offset+jj]);
                        iface->write_flash(block_addr+i+offset+jj,data[i+offset+jj]);
                    }
                else{
                    flash_vec_t tempo;
                    for(uint32_t j=0;j<32;j++){
                        tempo.push_back(data[i+offset+j]);
                    }
                    iface->write_flash(block_addr+i+offset,tempo);
//                    iface->write_flash_buffer(block_addr+i+offset,&(data[i+offset]));
                }
            }

            iface->write_flash(block_addr,0xD0);
            command_complete=0;
            while (!command_complete){
                status = iface->read_flash(0); // does address matter????
                command_complete=!!(status&0x80);  // check ready status
            }
            offset+=512;
        }

        iface->write_flash(0,0xFF);
        return true;
    }

    boost::uint16_t pax::micron_impl::micron_read(boost::uint32_t addr)
    {
        return iface->read_flash(addr);
    }
    boost::uint16_t pax::micron_impl::micron_read_array(boost::uint32_t addr)
    {
        iface->write_flash(0,0xFF);
        return iface->read_flash(addr);
    }

PAX_API pax::micron::sptr pax::micron::make(pax_iface::sptr iface)
  {
      return pax::micron::sptr(new pax::micron_impl(iface));
  }




