#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <gps.hpp>

namespace pax{
class gps_impl:public gps
{

public:
    gps_impl(pax_iface::sptr iface, boost::uint16_t addr):iface(iface),addr(addr)
    {
        running=false;
    }

    ~gps_impl(){
        stop();
    }

    float get_lat(){
        boost::lock_guard<boost::mutex> lock(_gps_mutex);
        return lat;
    }

    float get_lon(){
        boost::lock_guard<boost::mutex> lock(_gps_mutex);
        return lon;
    }
    float get_GMT_hour(){
        boost::lock_guard<boost::mutex> lock(_gps_mutex);
        return hour;
    }
    float get_GMT_minute(){
        boost::lock_guard<boost::mutex> lock(_gps_mutex);
        return minut;
    }

    float get_GMT_second(){
        boost::lock_guard<boost::mutex> lock(_gps_mutex);
        return seconds;
    }


    bool checksum(std::string a)
    {
        unsigned int i=1;
        char c=a[1];
        char check=0;
        while(i<a.size() && c!='*'){
            check=check^c;
            i++;
            c=a[i];
        }
        if(c!='*')
            return false;

        if(a.length()<(i+3))
            return false;
        //std::cout<<std::hex<<(unsigned int)(check)<<std::endl;
        char p=0;
        if((a[i+1]-'0')<0 ||(a[i+1]-'0')>9){
            if((a[i+1]-'A')<0 ||(a[i+1]-'A')>6)
                return false;
            p=(a[i+1]-'A'+10)<<4;
        }
        else{
            p=(a[i+1]-'0')<<4;
        }

        if((a[i+2]-'0')<0 ||(a[i+2]-'0')>9){
            if((a[i+2]-'A')<0 ||(a[i+2]-'A')>6)
                return false;
            p+=(a[i+2]-'A'+10);
        }
        else{
            p+=(a[i+2]-'0');
        }
        // std::cout<<"   "<<p <<"   "<<check <<std::endl;
        if(p==check)
            return true;
        else
            return false;
    }

    void get_data(std::string input)
    {
                buffer+=input;
               std::vector<std::string > SplitVec_fist;
                split( SplitVec_fist, buffer, boost::is_any_of("$"), boost::token_compress_on);

                for(size_t i=0;i<(SplitVec_fist.size()-1);i++)
                {
                        if(SplitVec_fist[i].find("GNGGA",0) !=std::string::npos)
                            enemei = enemei_t::GNGGA;
                        else if (SplitVec_fist[i].find("GPRMC",0) !=std::string::npos)
                            enemei = enemei_t::GPRMC;
                        else if (SplitVec_fist[i].find("GNGLL",0) !=std::string::npos)
                            enemei = enemei_t::GNGLL;
                        else {
                            enemei = enemei_t::UNKOWN;
                            continue;
                        }

                        std::vector<std::string > SplitVec;
                        split( SplitVec, SplitVec_fist[i], boost::is_any_of(","), boost::token_compress_on);
                        if(SplitVec.size()<6)
                            continue;
                        try{
                            switch(enemei){
                            case enemei_t::GNGGA:
                                lat =boost::lexical_cast<float>(SplitVec[2])/100;
                                lon =boost::lexical_cast<float>(SplitVec[4])/100;
                                UTC=boost::lexical_cast<float>(SplitVec[1]);
                                break;

                               case enemei_t::GPRMC:
                                lat =boost::lexical_cast<float>(SplitVec[3])/100;
                                lon =boost::lexical_cast<float>(SplitVec[5])/100;
                                UTC=boost::lexical_cast<float>(SplitVec[1]);
                                break;

                            case enemei_t::GNGLL:
                                lat =boost::lexical_cast<float>(SplitVec[1])/100;
                                lon =boost::lexical_cast<float>(SplitVec[3])/100;
                                UTC=boost::lexical_cast<float>(SplitVec[5]);
                                break;
                              default:
                                continue;
                                break;
                            }
                        } catch(...){continue;}

                        hour=UTC/10000;
                        minut=(UTC/100)%100;
                        seconds=UTC%100;
                        int lat_=lat;
                        lat=lat_+((lat-lat_)/60*100);
                        lat_=lon;
                        lon=lat_+((lon-lat_)/60*100);
                    }

                if(SplitVec_fist.size()>1)
                    buffer=SplitVec_fist[SplitVec_fist.size()-1];
                if(buffer.size() > 4000)
                    buffer.clear();
    }

    void gps_reader()
    {
        while(running){
            pax::byte_vector_t data_in(UART_MAX_LEN);
            data_in = iface->read_uart(addr,UART_MAX_LEN);
            //for(int i=0;i<data_in.size();i++)
            //std::cout<<(char)data_in[i];
            std::string sdata;
            sdata.assign(data_in.begin(),data_in.end());
            boost::replace_all(sdata,"\r","\n");
            if (data_in.size()>0){
                // boost::lock_guard<boost::mutex> lock(buff_mutex);
                buff += sdata;
                // boost::lock_guard<boost::mutex> lock(buff_mutex);
                get_data(buff);
            }
            else{
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }
        }
        boost::this_thread::interruption_point();
    }

    void run()
    {
        if(running==false){
            running=true;
            runner=boost::thread(boost::bind(&gps_impl::gps_reader,this));
        }
    }

    void stop()
    {
        running=false;
        //  boost::this_thread::sleep(boost::posix_time::millisec(10));
        if(runner.timed_join(boost::posix_time::millisec(1000)))
            runner.interrupt();
    }

private:
    typedef enum {
        GNGGA,
        GNGLL,
        GPRMC,
        UNKOWN
    }enemei_t;

    boost::mutex _gps_mutex;
    boost::mutex buff_mutex;
    pax_iface::sptr iface;
    boost::shared_ptr<boost::thread> reader;
    boost::uint16_t addr;
    double lat;
    double lon;
    double alt;
    int hour;
    int minut;
    int seconds;
    int UTC;
    enemei_t enemei;
    std::string buffer;
    std::string buff;
    bool running;
    boost::thread runner;

};

gps::sptr PAX_API gps::make(pax_iface::sptr iface,boost::uint16_t addr)
{
    gps::sptr _gps=sptr(new gps_impl(iface,addr));
    //_gps->run();
    return _gps;
}
}
