#include <pax_impl.hpp>
#include <serial.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <vector>
#include <servo.hpp>
#define MAX_LEN 20

boost::mutex buff_mutex;
std::string buff("");

void write_uart_thr(boost::shared_ptr<pax_iface> iface, uint32_t addr){
    std::string sdata;
    std::string temp="002500000010000";
    std::string temp2("123");
    temp2[0]=0xff;temp2[1]=0xaa;temp2[2]=0x55;
    sdata=temp2+temp;

    while (true){
        //std::cin >> sdata; // waits for endline itself?
        pax::byte_vector_t data(MAX_LEN);
        for(boost::uint32_t i=0;i<sdata.size();i+=MAX_LEN-1)
        {
            if (i+MAX_LEN-1 >=sdata.size())
            {
                data.resize(std::distance(sdata.begin()+i*(MAX_LEN-1),sdata.end()));
                std::copy(sdata.begin()+i*(MAX_LEN-1),sdata.end(),data.begin());
                data.push_back('\r');
            }
            else{
                std::copy(sdata.begin()+i*(MAX_LEN-1),sdata.end()+(i+1)*(MAX_LEN-1),data.begin());
                data.push_back('\r');
            }
            iface->write_uart(addr,data);
        }
    }
}

void read_uart_thr(boost::shared_ptr<pax_iface> iface, uint32_t addr){
    while(1)
    {
        pax::byte_vector_t data_in(MAX_LEN);
        data_in = iface->read_uart(addr,MAX_LEN);
        std::vector <std::string> fields;
        std::string sdata;
        sdata.assign(data_in.begin(),data_in.end());

        boost::replace_all(sdata,"\r","\n");
        if (data_in.size()>0){
            boost::lock_guard<boost::mutex> lock(buff_mutex);
            buff += sdata;
        }
        else
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        }
    }
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    mb_container_type tester;
    std::vector<boost::shared_ptr<pax::transport::sph::recv_packet_streamer> > streamers=pax_init(tester,2);


     return 0;

}






