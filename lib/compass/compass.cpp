#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <compass.hpp>

namespace pax{
class compass_impl:public compass
{

public:
    compass_impl(pax_iface::sptr iface, boost::uint16_t addr):iface(iface),addr(addr)
    {
        running = false;
        start();
    }

    ~compass_impl(){
        stop();
    }

    float get_heading(){
        boost::lock_guard<boost::mutex> lock(_heading_mutex);
        return heading;
    }

    void start(){
        boost::lock_guard<boost::mutex> lock(_write_mutex);
        compass_write("s\n");
    }

    std::string compass_read()
    {
        boost::lock_guard<boost::mutex> lock(_read_mutex);
        pax::byte_vector_t data_in(UART_MAX_LEN);
        pax::byte_vector_t data_in_tmp(UART_MAX_LEN);
        data_in.clear();
        while (running && data_in.back()!='\n'){
            if (data_in_tmp.size()==0)
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            data_in_tmp = iface->read_uart(addr,UART_MAX_LEN);
            data_in.insert(data_in.end(),data_in_tmp.begin(),data_in_tmp.end());
        }
        std::string value;
        value.assign(data_in.begin(),data_in.end());
        return value;
    }

    void compass_reader()
    {
        while(running)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            std::string msg=compass_read();
            while(running && msg.find("$HCHDM,")!=(size_t)0){
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                msg=compass_read();
            }
            {
                boost::lock_guard<boost::mutex> lock(_heading_mutex);
                std::string hd(msg.begin()+7,msg.begin()+12);
                heading = std::atof(hd.c_str());
            }
        }
        boost::this_thread::interruption_point();
    }

    void compass_write(std::string buf)
    {
        pax::byte_vector_t data(UART_MAX_LEN);

        for(boost::uint32_t i=0;i<buf.size();i+=UART_MAX_LEN)
        {
            if (i+UART_MAX_LEN >=buf.size())
            {
                data.resize(std::distance(buf.begin()+i*UART_MAX_LEN,buf.end()));
                std::copy(buf.begin()+i*UART_MAX_LEN,buf.end(),data.begin());
            }
            else{
                std::copy(buf.begin()+i*UART_MAX_LEN,buf.end()+(i+1)*UART_MAX_LEN,data.begin());
            }
            iface->write_uart(addr,data);
        }
    }

    void run(){
        if(running==false)
        {
            running=true;
            runner=boost::thread(boost::bind(&compass_impl::compass_reader,this));
        }
    }

    void stop()
    {
        running=false;
        if(runner.timed_join(boost::posix_time::millisec(1000)))
            runner.interrupt();
    }

private:
    boost::mutex _heading_mutex;
    boost::mutex _read_mutex;
    boost::mutex _write_mutex;
    pax_iface::sptr iface;
    boost::shared_ptr<boost::thread> reader;
    boost::thread runner;
    bool running;
    boost::uint16_t addr;
    float heading;

};

compass::sptr compass::make(pax_iface::sptr iface,boost::uint16_t addr)
{
    compass::sptr _compass=sptr(new compass_impl(iface,addr));
    //_compass->run();
    return _compass;
}
}
