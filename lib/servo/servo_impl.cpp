

#include "servo_impl.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <exception.hpp>
#include <iomanip>
#include <pax_regs.hpp>



pax::servo::sptr PAX_API pax::servo::make(pax_iface::sptr iface,boost::uint16_t addr){
    return sptr(new pax::servo_impl(iface,addr));
}




void pax::servo_impl::rotate_pos(double position,double speed=0.1)
{
    position=position-(std::floor(position/360)*360);
    unsigned int iPosition=position*10000.0/360.0;

    std::string header("    ");
    header[0]=0xff;
    header[1]=0xaa;
    header[2]=0x55;
    header[3]='0';

    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << (iPosition);
    ss << std::setw(5) << std::setfill('0') << ((int)(speed*10));
    ss << std::setw(4) << std::setfill('0') << (0);
    std::string str = header+ss.str();
    pax::byte_vector_t command;
    std::copy( str.begin(), str.end(), std::back_inserter(command));
    iface->write_uart(1,command);



   // std::string str_speed=boost::lexical_cast<std::string>((boost::uint32_t)speed*10);


}

void pax::servo_impl::rotate_cw(double speed=0.1)
{
    double  position=0.0f;
    position=position-(std::floor(position/360)*360);
    unsigned int iPosition=position*10000.0/360.0;

    std::string header("    ");
    header[0]=0xff;
    header[1]=0xaa;
    header[2]=0x55;
    header[3]='1';

    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << (iPosition);
    ss << std::setw(5) << std::setfill('0') << ((int)(speed*10));
    ss << std::setw(4) << std::setfill('0') << (0);
    std::string str = header+ss.str();
    pax::byte_vector_t command;
    std::copy( str.begin(), str.end(), std::back_inserter(command));
    iface->write_uart(1,command);


}

void pax::servo_impl::rotate_ccw(double speed=0.1)
{
    double  position=0.0f;
    position=position-(std::floor(position/360)*360);
    unsigned int iPosition=position*10000.0/360.0;

    std::string header("    ");
    header[0]=0xff;
    header[1]=0xaa;
    header[2]=0x55;
    header[3]='2';

    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << (iPosition);
    ss << std::setw(5) << std::setfill('0') << ((int)(speed*10));
    ss << std::setw(4) << std::setfill('0') << (0);
    std::string str = header+ss.str();
    pax::byte_vector_t command;
    std::copy( str.begin(), str.end(), std::back_inserter(command));
    iface->write_uart(1,command);
}

void pax::servo_impl::rotate_speed_sector(double position,double delta_pos=20,double speed=.1)
{
    position=position-(std::floor(position/360)*360);
     delta_pos=delta_pos-(std::floor(delta_pos/360)*360);
     unsigned int idelta_pos=delta_pos*5000/360;
    unsigned int iPosition=position*10000.0/360.0;

    std::string header("    ");
    header[0]=0xff;
    header[1]=0xaa;
    header[2]=0x55;
    header[3]='3';

    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << (iPosition);
    ss << std::setw(5) << std::setfill('0') << ((int)(speed*10));
    ss << std::setw(4) << std::setfill('0') << (idelta_pos);
    std::string str = header+ss.str();
    pax::byte_vector_t command;
    std::copy( str.begin(), str.end(), std::back_inserter(command));
    iface->write_uart(1,command);
}

double  pax::servo_impl::get_shaft_angel()
{
    return angle;
}

void pax::servo_impl::read_data()
{
    pax::byte_vector_t read_buff;
    pax::byte_vector_t buff;
    std::string ss;
    while(running)
    {
        buff=iface->read_uart(1,20);
        std::copy(buff.begin(),buff.end(),std::back_inserter(read_buff));
        if(read_buff.size()>36)
        {
            for(int i=0;i<18;i++)
                if( read_buff[i+1]==0xAA && read_buff[i+2]==0x55)
                {
                    ss.clear();
                    std::copy(read_buff.begin()+4+i,read_buff.begin()+9+i,std::back_inserter(ss));
                    try {
                        angle=boost::lexical_cast<double>(ss)*360.0f/10000.0f;
                       // std::cout<<angle<<std::endl;

                    } catch (...) {
                        read_buff.clear();
                    }

                }
            read_buff.erase(read_buff.begin(),read_buff.end()-19);
        }
        if(buff.size()==0)
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));


    }
}



bool pax::servo_impl::run()
{
    if(running==false){
        running=true;
        runner=boost::thread(boost::bind(&servo_impl::read_data,this));
    }
	return true;
}

bool pax::servo_impl::stop()
{
    running=false;
    //  boost::this_thread::sleep(boost::posix_time::millisec(10));
    if(runner.timed_join(boost::posix_time::millisec(1000))){
        runner.interrupt();
    }

	return true;
}

