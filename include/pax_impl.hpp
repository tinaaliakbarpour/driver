#ifndef PAX_IMPL
#define PAX_IMPL
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <device_addr.hpp>
#include <udp_simple.hpp>
#include <device_addr.hpp>
#include <if_addrs.hpp>
#include <paxtype.h>
#include <iostream>
#include <fstream>
#include <fw_common.h>
#include <utility.hpp>
#include <pax_iface.h>
#include <pax_regs.hpp>
#include <rx_dsp_core_200.hpp>
#include <vrt_if_packet.hpp>
#include <super_recv_packet_handler.hpp>
#include <super_send_packet_handler.hpp>
#include <usrp2_fifo_ctrl.hpp>
#include <rx_frontend_core_200.hpp>
#include <udp_zero_copy.hpp>
#include <zero_copy.hpp>
#include <ad9361_client.h>
#include <ad9361_ctrl.hpp>
#include <micron.hpp>
#include <pax_sync.h>
#include <compass.hpp>
#include <gps.hpp>
#include <tx_dsp_core_200.hpp>
#include <boost/thread/condition.hpp>
#include <bounded_buffer.hpp>
#include <thread.hpp>
#include <async_packet_handler.hpp>
#include <time64_core_200.hpp>
#include <filter_bank.hpp>
#include <eeprom.hpp>
#include <spi_wb_iface.hpp>
#include <ad9361_device.h>
#include <filter_bank_imple.hpp>

static const size_t DEFAULT_NUM_FRAMES = 32;
static const double USRP2_LINK_RATE_BPS = 1000e6/8;
static const boost::uint32_t USRP2_RX_SID_BASE = 3;


/***********************************************************************
 * flow control monitor for a single tx channel
 *  - the pirate thread calls update
 *  - the get send buffer calls check
 **********************************************************************/
class flow_control_monitor{
public:
    typedef boost::uint32_t seq_type;
    typedef boost::shared_ptr<flow_control_monitor> sptr;

    /*!
     * Make a new flow control monitor.
     * \param max_seqs_out num seqs before throttling
     */
    flow_control_monitor(seq_type max_seqs_out):_max_seqs_out(max_seqs_out){
        this->clear();
        _ready_fcn = boost::bind(&flow_control_monitor::ready, this);
    }

    //! Clear the monitor, Ex: when a streamer is created
    void clear(void){
        _last_seq_out = 0;
        _last_seq_ack = 0;
    }

    /*!
     * Gets the current sequence number to go out.
     * Increments the sequence for the next call
     * \return the sequence to be sent to the dsp
     */
   PAX_INLINE seq_type get_curr_seq_out(void){
        return _last_seq_out++;
    }

    /*!
     * Check the flow control condition.
     * \param timeout the timeout in seconds
     * \return false on timeout
     */
    PAX_INLINE bool check_fc_condition(double timeout){
        boost::mutex::scoped_lock lock(_fc_mutex);
        if (this->ready()) return true;
        boost::this_thread::disable_interruption di; //disable because the wait can throw
        return _fc_cond.timed_wait(lock, boost::posix_time::microseconds(long(timeout*1e6)), _ready_fcn);
    }

    /*!
     * Update the flow control condition.
     * \param seq the last sequence number to be ACK'd
     */
    PAX_INLINE void update_fc_condition(seq_type seq){
        boost::mutex::scoped_lock lock(_fc_mutex);
        _last_seq_ack = seq;
        lock.unlock();
        _fc_cond.notify_one();
    }

private:
    bool ready(void){
        return seq_type(_last_seq_out -_last_seq_ack) < _max_seqs_out;
    }

    boost::mutex _fc_mutex;
    boost::condition _fc_cond;
    seq_type _last_seq_out, _last_seq_ack;
    const seq_type _max_seqs_out;
    boost::function<bool(void)> _ready_fcn;
};



/***********************************************************************
 * io impl details (internal to this file)
 * - pirate crew
 * - alignment buffer
 * - thread loop
 * - vrt packet handler states
 **********************************************************************/
struct io_impl{

    io_impl(void):
        async_msg_fifo(1000/*messages deep*/),
        tick_rate(1 /*non-zero default*/)
    {
        /* NOP */
    }

    ~io_impl(void){
        //Manually deconstuct the tasks, since this was not happening automatically.
        pirate_tasks.clear();
    }

    pax::transport::managed_send_buffer::sptr get_send_buff(size_t chan, double timeout){
        flow_control_monitor &fc_mon = *fc_mons[chan];

        //wait on flow control w/ timeout
        if (not fc_mon.check_fc_condition(timeout)) return pax::transport::managed_send_buffer::sptr();

        //get a buffer from the transport w/ timeout
        pax::transport::managed_send_buffer::sptr buff = tx_xports[chan]->get_send_buff(timeout);

        //write the flow control word into the buffer
        if (buff.get()) buff->cast<boost::uint32_t *>()[0] = pax::htonx(fc_mon.get_curr_seq_out());

        return buff;
    }

    //tx dsp: xports and flow control monitors
    std::vector<pax::transport::zero_copy_if::sptr> tx_xports;
    std::vector<flow_control_monitor::sptr> fc_mons;

    //methods and variables for the pirate crew
    void recv_pirate_loop(pax::transport::zero_copy_if::sptr, size_t);
    std::list<pax::task::sptr> pirate_tasks;
    pax::transport::bounded_buffer<pax::async_metadata_t> async_msg_fifo;
    double tick_rate;
};




struct mb_container_type{
    pax_iface::sptr iface;
    usrp2_fifo_ctrl::sptr fifo_ctrl;
    spi_wb_iface::sptr spi_and_wb_iface;
    pax::spi_iface::sptr spiface;
    pax::wb_iface::sptr wbiface;
    std::vector<pax::usrp::ad9361_ctrl::sptr> ad_9361;
    pax::usrp::ad9361_ctrl::sptr synthesizer;
    std::vector<rx_dsp_core_200::sptr> rx_dsps;
    std::vector<boost::shared_ptr<pax::rx_streamer> > rx_streamers;
    std::vector< boost::shared_ptr<pax::transport::sph::send_packet_streamer> > tx_streamers;
    std::vector<tx_dsp_core_200::sptr> tx_dsp;
    time64_core_200::sptr time64;
    std::vector<pax::transport::zero_copy_if::sptr> rx_dsp_xports;
    std::vector<pax::transport::zero_copy_if::sptr> tx_dsp_xports;
    pax::transport::zero_copy_if::sptr fifo_ctrl_xport;
    std::vector<boost::shared_ptr<io_impl> >  _io_impls;
    size_t rx_chan_occ, tx_chan_occ;
    pax::pax_sync::sptr sync;
    pax::compass::sptr compass;
    pax::gps::sptr gps;
    std::vector<pax::filter_bank::sptr> filter_bank;
    pax::usrp::main_board_t board ;
    pax::usrp::daughter_t daughter_b;
    uint32_t N_AD9361;
    uint32_t N_AD9364;
    pax::usrp::ad_ref_clk_t ad_ref_clk;
    //usrp2_clock_ctrl::sptr clock;
    //usrp2_codec_ctrl::sptr codec;
    //pax::gps_ctrl::sptr gps;
    //rx_frontend_core_200::sptr rx_fe;
    //tx_frontend_core_200::sptr tx_fe;
    //   user_settings_core_200::sptr user;
    //pax::usrp::dboard_manager::sptr dboard_manager;
    //pax::usrp::dboard_iface::sptr dboard_iface;

    mb_container_type(void) : rx_chan_occ(0), tx_chan_occ(0) { rx_streamers.resize(0); }
};


/***********************************************************************
 * Stream destination programmer
 **********************************************************************/
namespace pax {namespace usrp {


class pax_ad9361_client_t : public ad9361_params {
    boost::uint32_t switchs;
    daughter_t db;
    main_board_t mb;
    digital_interface_mode_t digital_interface_mode;
    pax_iface::sptr iface;
    boost::shared_ptr<filter_bank> flt;// PH
public:
    ~pax_ad9361_client_t();

    void set_filter_bank(double freq, std::string direction);
    void set_filter_bank(boost::shared_ptr<filter_bank> filter); // PH

    void set_addidtional_register(frequency_band_t band , uint8_t which_ad9361);
    void set_interface(pax_iface::sptr _iface);
    double get_band_edge(frequency_band_t band) ;
    clocking_mode_t get_clocking_mode();
    digital_interface_mode_t get_digital_interface_mode();
    digital_interface_delays_t get_digital_interface_timing(size_t which);
    boost::uint32_t get_switch();
    void set_switch(boost::uint32_t sw);
    void set_mb(main_board_t mboard);
    void set_db(daughter_t dboard);

private:
    double last_set_freq=5.5e9; // PH
    double get_filterbank_edge_sky_v1(filterbank_band_t band);
    boost::uint32_t get_filter_bank_sw_sky_v1(double freq);
};
}
              }
typedef boost::shared_ptr<pax::transport::sph::recv_packet_streamer> streamer_t;
typedef std::vector<boost::shared_ptr<pax::transport::sph::recv_packet_streamer> > vec_streamers_t;
typedef struct {
    mb_container_type dev;
    std::vector<boost::shared_ptr<pax::transport::sph::recv_packet_streamer> >  streamers;
    pax::device_addr_t pax_dev_addr;
} pax_dev_t;
typedef std::vector<pax_dev_t> vec_pax_dev_t;
typedef std::vector<std::string> vec_IP_t;


PAX_API vec_IP_t find_all_pax_ip();
PAX_API pax::device_addr_t pax_find(pax::eth_ip_addr_t IP);
PAX_API pax_dev_t pax_init(size_t N_STREAM,std::string IP ,  bool bypass_board_specefic_init = false);
PAX_API pax_dev_t pax_init(size_t N_STREAM,pax::device_addr_t hint ,  bool bypass_board_specefic_init = false);
PAX_API vec_streamers_t pax_init(mb_container_type& tester,size_t  N_STREAM ,  bool bypass_board_specefic_init = false);
PAX_API vec_streamers_t pax_init(mb_container_type& tester,size_t  N_STREAM,
                                 pax::device_addr_t hint , bool bypass_board_specefic_init = false);


void net_work_init(mb_container_type& tester,pax::device_addrs_t& out,int N_STREAM);
vec_streamers_t set_streams(mb_container_type& tester,pax::device_addrs_t& out,int N_STREAM);
void init_time(mb_container_type& tester);
void read_fw_and_init(mb_container_type& tester);
std::string read_flash(mb_container_type& tester);
void set_board_type(mb_container_type& tester);
uint32_t set_daughter_board(mb_container_type& tester);
void init_ad9361(mb_container_type& tester,uint32_t N_AD9361);
boost::uint32_t set_num_ad9361(mb_container_type& tester);
void chech_incompatible_state(mb_container_type& tester);

extern PAX_API mb_container_type pax_radio;

#endif
