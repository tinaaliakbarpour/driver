#ifndef __RECORDER__
#define __RECORDER__

#include <pax_impl.hpp>
#include <eeprom.hpp>
#include <boost/weak_ptr.hpp>
class recorder :public boost::noncopyable
{
public:
    typedef boost::shared_ptr<recorder> sptr;
    static sptr make(boost::shared_ptr<pax::transport::sph::recv_packet_streamer> stream,int number, mb_container_type tester, int num_log=10000);
    virtual void run()=0;
};

class recorder_impl :public recorder
{
    boost::shared_ptr<pax::transport::sph::recv_packet_streamer> stream;
    int number, num_log;
    boost::shared_ptr<std::ofstream> myFile;//("data1.bin", std::ios::out | std::ios::binary)
    mb_container_type tester;
public:
    recorder_impl(boost::shared_ptr<pax::transport::sph::recv_packet_streamer> stream, int number, int num_log, mb_container_type tester);
    void run();
};


#endif
