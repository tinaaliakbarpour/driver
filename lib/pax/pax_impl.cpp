#include <pax_impl.hpp>
#include <ADF4351.h>
#include <pax_sync.h>
#include <boost/weak_ptr.hpp>
#include <filter_bank.hpp>
#include <string>
#include <math.h>
#include <boost/algorithm/string/replace.hpp>
#include <log.hpp>

//#include <filter_bank_imple.hpp>


pax::usrp::pax_ad9361_client_t::~pax_ad9361_client_t() {}
void pax::usrp::pax_ad9361_client_t::set_interface(pax_iface::sptr _iface){
    iface= _iface;
}
void pax::usrp::pax_ad9361_client_t::set_filter_bank(double freq, std::string direction){
    boost::uint32_t band ;
    if(!flt)
        return;
    switch (db) {
    case pax2:
        band = get_filter_bank_sw_sky_v1(freq);

        if (band != get_filter_bank_sw_sky_v1(last_set_freq)){
            iface->poke32(U2_REG_SR_ADDR(SR_FILT_SW),band);
        }
        last_set_freq = freq;
        break;

    #ifdef __HAND_OFF__
    case pax2s6:
        if(direction == "RX")
            flt->set_filter_path(freq, direction,false);
        break;
    #endif
    case pax2_9361_filter_bank:
        if(direction == "RX")
            flt->set_filter_path(freq, direction,false); // PH
        break;
    case pax8v7_9361_filter_bank:
        if(direction == "RX")
            flt->set_filter_path(freq, direction,false);
        break;
    case pax8_gnss_8ch_monitoring:
            flt->set_filter_path(freq, direction,false);
        break;
    default:
            flt->set_filter_path(freq, direction,false /*changed from true */); //this one has to be false every time because if true the tune function call again 

        // break;
    }

}

void pax::usrp::pax_ad9361_client_t::set_filter_bank(boost::shared_ptr<filter_bank> filter){ // PH
    flt = filter;
}



boost::uint32_t pax::usrp::pax_ad9361_client_t::get_filter_bank_sw_sky_v1(double freq){
    if (freq < get_filterbank_edge_sky_v1(FILTER_BANK_BAND0)) {
        return 0x3;
    }else if ((freq
               >= get_filterbank_edge_sky_v1(FILTER_BANK_BAND0))
              && (freq
                  < get_filterbank_edge_sky_v1(FILTER_BANK_BAND1))) {
        return 0x2;
    }else if ((freq
               >= get_filterbank_edge_sky_v1(FILTER_BANK_BAND1))
              && (freq
                  < get_filterbank_edge_sky_v1(FILTER_BANK_BAND2))) {
        return 0x1;
    }else if ((freq
               >= get_filterbank_edge_sky_v1(FILTER_BANK_BAND2))
              && (freq
                  < get_filterbank_edge_sky_v1(FILTER_BANK_BAND3))) {
        return 0x0;
    }else if (freq >= get_filterbank_edge_sky_v1(FILTER_BANK_BAND3)){
        std::cout << "Warning: Frequency is out of input filter bands.\nCURR=" << freq <<
                     "\nMAX=" <<  get_filterbank_edge_sky_v1(FILTER_BANK_BAND3) << std::endl;
        return 0x0;
    }else
        return 0x0;
}


double pax::usrp::pax_ad9361_client_t::get_filterbank_edge_sky_v1(filterbank_band_t band) {
    switch (band) {
    case NO_FILTER_BANK:      return -1;
    case FILTER_BANK_BAND0:   return 200e6;
    case FILTER_BANK_BAND1:   return 440e6;
    case FILTER_BANK_BAND2:   return 800e6;
    case FILTER_BANK_BAND3:   return 1250e6;
    default:                return 0;
    }
}
void pax::usrp::pax_ad9361_client_t::set_addidtional_register(frequency_band_t band , uint8_t which_ad9361){
    switch(db){
    case pax8 :
        switch (band) {
        case AD9361_RX_BAND0_MAX_LIMIT:
            iface->poke32(U2_REG_SR_ADDR(PAX8_SR_RX_PORT_ADDR_WHICH_IC(which_ad9361)),3);
            break;
        case AD9361_RX_BAND1_MAX_LIMIT:
            iface->poke32(U2_REG_SR_ADDR(PAX8_SR_RX_PORT_ADDR_WHICH_IC(which_ad9361)),1);
            break;
        case AD9361_RX_BAND2_MAX_LIMIT:
            iface->poke32(U2_REG_SR_ADDR(PAX8_SR_RX_PORT_ADDR_WHICH_IC(which_ad9361)),2);
            break;
        case AD9361_TX_BAND0_MAX_LIMIT:
            iface->poke32(U2_REG_SR_ADDR(SR_TX_SW),1);
            break;
        case AD9361_TX_BAND1_MAX_LIMIT:
            iface->poke32(U2_REG_SR_ADDR(SR_TX_SW),3);
            break;
        default:
            throw runtime_error("this value is not supported");
        }
        break;

    default:{ return;}
    }
}
double pax::usrp::pax_ad9361_client_t::get_band_edge(frequency_band_t band) {
    switch(db){
    case daughter_t::pax8:
        switch (band) {
        case AD9361_RX_BAND0_MAX_LIMIT: return 2.2e9;
        case AD9361_RX_BAND1_MAX_LIMIT: return 4e9;
        case AD9361_RX_BAND2_MAX_LIMIT: return 6e9;
        case AD9361_TX_BAND0_MAX_LIMIT: return 2.5e9;
        case AD9361_TX_BAND1_MAX_LIMIT: return 6e9;
        default:    throw runtime_error("this value is not supported");
        }
        break;
    case daughter_t::pax2:
        return 6e9;

     break;
    default:{   return -1; }
    }
}


pax::usrp::clocking_mode_t pax::usrp::pax_ad9361_client_t::get_clocking_mode() {
    if(db==Analog)
        return AD9361_XTAL_P_CLK_PATH;
    else
        return AD9361_XTAL_N_CLK_PATH;
}

pax::usrp::digital_interface_mode_t pax::usrp::pax_ad9361_client_t::get_digital_interface_mode() {

    switch(db){
    case Analog :                       {return AD9361_DDR_FDD_LVDS_2R_2T ;}break;
    case pax8 :                         {return AD9361_DDR_FDD_LVDS_2R_2T ;}break;
    case pax2:                          {return AD9361_DDR_FDD_LVDS_2R_2T ;}break;
    case pax2s6:                        {return AD9361_DDR_FDD_LVCMOS_1R_1T;}break;
    case pax8_gnss_4ch:                 {return AD9361_DDR_FDD_LVDS_1R_1T ;}break;
    case pax8_gnss_8ch:                 {return AD9361_DDR_FDD_LVDS_1R_1T ;}break;
    case pax8_gnss_8ch_calibrasion:     {return AD9361_DDR_FDD_LVDS_1R_1T ;}break;
    case pax8_gnss_8ch_monitoring:      {return AD9361_DDR_FDD_LVDS_2R_2T ;}break;
    case pax2_9361_filter_bank:         {return AD9361_DDR_FDD_LVDS_2R_2T ;}break;
    case pax8v7_9361_filter_bank:       {return AD9361_DDR_FDD_LVDS_2R_2T_Virtex ;}break;
    case NULL_B:                        {return AD9361_UNKOWN;}break;
    default :                           {return AD9361_UNKOWN;}break;
    }


}

pax::usrp::digital_interface_delays_t pax::usrp::pax_ad9361_client_t::get_digital_interface_timing(size_t which) {
    digital_interface_delays_t delays;
    delays.rx_clk_delay = which;
    // 0.3 ns = LSB
    switch(get_digital_interface_mode()){
    case AD9361_DDR_FDD_LVCMOS_1R_1T : { //this delay adjust for maximom clock rate
        delays.rx_clk_delay = 0x0;
        delays.rx_data_delay = 0xd;
        delays.tx_clk_delay = 0x0;
        delays.tx_data_delay =0xd;
    }break;
    case AD9361_DDR_FDD_LVCMOS_2R_2T : {
        delays.rx_clk_delay = 0x5;
        delays.rx_data_delay = 0x0;
        delays.tx_clk_delay = 0x0;
        delays.tx_data_delay =0x6;
    }break;
    case AD9361_DDR_FDD_LVDS_1R_1T : {
        delays.rx_clk_delay  = 0x5;
        delays.rx_data_delay = 0x0;
        delays.tx_clk_delay  = 0x0;
        delays.tx_data_delay  =0x6;
    }break;
    case AD9361_DDR_FDD_LVDS_2R_2T : {
        delays.rx_clk_delay = 0x4;
        delays.rx_data_delay = 0x0;
        delays.tx_clk_delay = 0x0;
        delays.tx_data_delay =0x3;
    }break;
    case AD9361_DDR_FDD_LVDS_2R_2T_Virtex : {
        delays.rx_clk_delay = 0x6;//5 //0
        delays.rx_data_delay = 0x0;//0 //b
        delays.tx_clk_delay = 0x0;//4 //2
        delays.tx_data_delay =0x4;//0 //0
    }break;
    default :{
        delays.rx_clk_delay = 0x4;
        delays.rx_data_delay = 0x0;
        delays.tx_clk_delay = 0x0;
        delays.tx_data_delay =0x3;
    }
    }

    return delays;
}

boost::uint32_t pax::usrp::pax_ad9361_client_t::get_switch()
{
    return switchs;
}

void pax::usrp::pax_ad9361_client_t::set_switch(boost::uint32_t sw)
{
    switchs = sw;
}

void pax::usrp::pax_ad9361_client_t::set_mb(main_board_t mboard){
    mb = mboard;
}

void pax::usrp::pax_ad9361_client_t::set_db(daughter_t dboard){
    db = dboard;
}


static pax::transport::zero_copy_if::sptr make_xport(
        const std::string &addr,
        const std::string &port,
        const pax::device_addr_t &hints,
        const std::string &filter
        ){

    //only copy hints that contain the filter word
    pax::device_addr_t filtered_hints;
    BOOST_FOREACH(const std::string &key, hints.keys()){
        if (key.find(filter) == std::string::npos) continue;
        filtered_hints[key] = hints[key];
    }

    pax::transport::zero_copy_xport_params default_buff_args;
    default_buff_args.send_frame_size = pax::transport::udp_simple::mtu;
    default_buff_args.recv_frame_size = pax::transport::udp_simple::mtu;
    default_buff_args.num_send_frames = DEFAULT_NUM_FRAMES;
    default_buff_args.num_recv_frames = DEFAULT_NUM_FRAMES;

    //make the transport object with the filtered hints
    pax::transport::udp_zero_copy::buff_params ignored_params;
    pax::transport::zero_copy_if::sptr xport = pax::transport::udp_zero_copy::make(addr, port, default_buff_args, ignored_params, filtered_hints);

    //Send a small data packet so the usrp2 knows the udp source port.
    //This setup must happen before further initialization occurs
    //or the async update packets will cause ICMP destination unreachable.
    static const boost::uint32_t data[2] = {
        pax::htonx(boost::uint32_t(0 /* don't care seq num */)),
        pax::htonx(boost::uint32_t(USRP2_INVALID_VRT_HEADER))
    };
    pax::transport::managed_send_buffer::sptr send_buff = xport->get_send_buff();
    std::memcpy(send_buff->cast<void*>(), &data, sizeof(data));
    send_buff->commit(sizeof(data));

    return xport;
}

void program_stream_dest(
        pax::transport::zero_copy_if::sptr &xport, const pax::stream_args_t &args
        ){
    //perform an initial flush of transport
    while (xport->get_recv_buff(0.0)){}

    //program the stream command
    usrp2_stream_ctrl_t stream_ctrl = usrp2_stream_ctrl_t();
    stream_ctrl.sequence = pax::htonx(boost::uint32_t(0 /* don't care seq num */));
    stream_ctrl.vrt_hdr = pax::htonx(boost::uint32_t(USRP2_INVALID_VRT_HEADER));

    //user has provided an alternative address and port for destination
    if (args.args.has_key("addr") and args.args.has_key("port")){
       PAX_LOGGER_INFO("PAX")
            << boost::format("Programming streaming destination for custom address. "
                             "IPv4 Address: %s, UDP Port: %s")
                   % args.args["addr"] % args.args["port"];

        boost::asio::io_service io_service;
        boost::asio::ip::udp::resolver resolver(io_service);
        boost::asio::ip::udp::resolver::query query( boost::asio::ip::udp::v4(), args.args["addr"], args.args["port"]);
        boost::asio::ip::udp::endpoint endpoint = *resolver.resolve(query);
        stream_ctrl.ip_addr = pax::htonx(boost::uint32_t(endpoint.address().to_v4().to_ulong()));
        stream_ctrl.udp_port = pax::htonx(boost::uint32_t(endpoint.port()));

        for (size_t i = 0; i < 3; i++){
            std::cout<< "ARP attempt " << i << std::endl;
            pax::transport::managed_send_buffer::sptr send_buff = xport->get_send_buff();
            std::memcpy(send_buff->cast<void *>(), &stream_ctrl, sizeof(stream_ctrl));
            send_buff->commit(sizeof(stream_ctrl));
            send_buff.reset();
            boost::this_thread::sleep(boost::posix_time::milliseconds(300));
            //   pax::transport::managed_recv_buffer::sptr recv_buff = xport->get_recv_buff(0.0);
            //            if (recv_buff and recv_buff->size() >= sizeof(boost::uint32_t)){
            //                const boost::uint32_t result = pax::ntohx(recv_buff->cast<const boost::uint32_t *>()[0]);
            //                if (result == 0){
            //                    std::cout<< "Success! " << std::endl;
            //                    return;
            //                }
            //            }
        }
        throw pax::runtime_error("Device failed to ARP when programming alternative streaming destination.");
    }

    else{
        //send the partial stream control without destination
        pax::transport::managed_send_buffer::sptr send_buff = xport->get_send_buff();
        std::memcpy(send_buff->cast<void *>(), &stream_ctrl, sizeof(stream_ctrl));
        send_buff->commit(sizeof(stream_ctrl)/2);
    }
}


boost::shared_ptr<pax::transport::sph::recv_packet_streamer> get_rx_stream(const pax::stream_args_t &args_,mb_container_type& mb,int dsp){
    pax::stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    //calculate packet size
    static const size_t hdr_size = 0
            + pax::transport::vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
            + sizeof(pax::transport::vrt::if_packet_info_t().tlr) //forced to have trailer
            - sizeof(pax::transport::vrt::if_packet_info_t().cid) //no class id ever used
            - sizeof(pax::transport::vrt::if_packet_info_t().tsi) //no int time ever used
            ;
    const size_t bpp = mb.rx_dsp_xports[dsp]->get_recv_frame_size() - hdr_size;
    const size_t bpi =4;// pax::convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

    /// PH starred

    //make the new streamer given the samples per packet
    boost::shared_ptr<pax::transport::sph::recv_packet_streamer> my_streamer = boost::make_shared<pax::transport::sph::recv_packet_streamer>(spp);

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_unpacker(&pax::transport::vrt::if_hdr_unpack_be);

    //set the converter
    pax::convert::id_type id;
    id.num_inputs = 1;
    id.num_outputs = 1;

    id.input_format  = "f32";
    id.output_format = "f32_item32_le";
    my_streamer->set_converter(id);

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far =1;

        num_chan_so_far += mb.rx_chan_occ;
        if (chan < num_chan_so_far){
            mb.rx_dsps[dsp]->set_nsamps_per_packet(spp); //seems to be a good place to set this
            mb.rx_dsps[dsp]->setup(args);
            program_stream_dest(mb.rx_dsp_xports[dsp], args);
            my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
                                                     &pax::transport::zero_copy_if::get_recv_buff, mb.rx_dsp_xports[dsp], _1
                                                     ), true /*flush*/);


            my_streamer->set_issue_stream_cmd(chan_i, boost::bind(
                                                  &rx_dsp_core_200::issue_stream_command, mb.rx_dsps[dsp], _1));
            mb.rx_streamers[dsp] = my_streamer; //store weak pointer

        }
    }


    //set the packet threshold to be an entire socket buffer's worth
    const size_t packets_per_sock_buff = size_t(50e6/mb.rx_dsp_xports[dsp]->get_recv_frame_size());
    my_streamer->set_alignment_failure_threshold(packets_per_sock_buff);

    //sets all tick and samp rates on this streamer
    // this->update_rates();

    return my_streamer;
}


/***********************************************************************
 * Transmit streamer
 **********************************************************************/
boost::shared_ptr<pax::transport::sph::send_packet_streamer> get_tx_stream(const pax::stream_args_t &args_, boost::shared_ptr<io_impl> _io_impl,mb_container_type& mb,int dsp_no)
{
    pax::stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    //calculate packet size
    static const size_t vrt_send_header_offset_words32 = 1;
    static const size_t hdr_size = 0
            +vrt_send_header_offset_words32*sizeof(boost::uint32_t)
            +pax::transport::vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
            + sizeof(pax::transport::vrt::if_packet_info_t().tlr) //forced to have trailer
            - sizeof(pax::transport::vrt::if_packet_info_t().cid) //no class id ever used
            - sizeof(pax::transport::vrt::if_packet_info_t().sid) //no stream id ever used
            - sizeof(pax::transport::vrt::if_packet_info_t().tsi) //no int time ever used
            ;
    const size_t bpp = mb.tx_dsp_xports[dsp_no]->get_send_frame_size() - hdr_size;
    const size_t spp = bpp/4;//pax::convert::get_bytes_per_item(args.otw_format);

    //make the new streamer given the samples per packet
    boost::shared_ptr<pax::transport::sph::send_packet_streamer> my_streamer = boost::make_shared<pax::transport::sph::send_packet_streamer>(spp);

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_packer(&pax::transport::vrt::if_hdr_pack_be, vrt_send_header_offset_words32);

    //set the converter
    pax::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = 1;
    id.output_format = args.otw_format + "_item32_be";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++){
        const size_t chan = args.channels[chan_i];
        size_t num_chan_so_far = 0;
        size_t abs = 0;
        //   BOOST_FOREACH(const std::string &mb, _mbc.keys()){
        num_chan_so_far += mb.tx_chan_occ;
        if (chan < 1){
            //            const size_t dsp = chan + mb.tx_chan_occ - num_chan_so_far;
            if (not args.args.has_key("noclear")){
                _io_impl->fc_mons[abs]->clear();
            }
            mb.tx_dsp[dsp_no]->clear();
            mb.tx_dsp[dsp_no]->setup(args);
            mb.tx_dsp[dsp_no]->set_updates(0,5);
            my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
                                                     &io_impl::get_send_buff, _io_impl.get(), abs, _1
                                                     ));
            my_streamer->set_async_receiver(boost::bind(&pax::transport::bounded_buffer<pax::async_metadata_t>::pop_with_timed_wait, &(_io_impl->async_msg_fifo), _1, _2));
            mb.tx_streamers[dsp_no] = my_streamer; //store weak pointer
            break;
        }
        abs += 1; //assume 1 tx dsp
        // }
    }

    //sets all tick and samp rates on this streamer
    //this->update_rates();

    return my_streamer;
}


/***********************************************************************
  * Receive Pirate Loop
  * - while raiding, loot for message packet
  * - update flow control condition count
  * - put async message packets into queue
  **********************************************************************/
void io_impl::recv_pirate_loop(
        pax::transport::zero_copy_if::sptr err_xport, size_t index
        ){
    //    static const boost::uint32_t USRP2_TX_ASYNC_SID = 2;
    pax::set_thread_priority_safe(1,true);

    //store a reference to the flow control monitor (offset by max dsps)
    flow_control_monitor &fc_mon = *(this->fc_mons[index]);

    while (not boost::this_thread::interruption_requested()){
        pax::transport::managed_recv_buffer::sptr buff = err_xport->get_recv_buff();
        if (not buff.get()) continue; //ignore timeout/error buffers

        try{
            //extract the vrt header packet info
            pax::transport::vrt::if_packet_info_t if_packet_info;
            if_packet_info.num_packet_words32 = buff->size()/sizeof(boost::uint32_t);
            const boost::uint32_t *vrt_hdr = buff->cast<const boost::uint32_t *>();
            pax::transport::vrt::if_hdr_unpack_be(vrt_hdr, if_packet_info);

            //handle a tx async report message
            if (if_packet_info.sid == 0 and if_packet_info.packet_type != pax::transport::vrt::if_packet_info_t::PACKET_TYPE_DATA){

                //fill in the async metadata
                pax::async_metadata_t metadata;
                pax::pax2::load_metadata_from_buff(pax::ntohx<boost::uint32_t>, metadata, if_packet_info, vrt_hdr, tick_rate, index);

                //catch the flow control packets and react
                if (metadata.event_code == 0){
                    boost::uint32_t fc_word32 = (vrt_hdr + if_packet_info.num_header_words32)[1];
                    fc_mon.update_fc_condition(pax::ntohx(fc_word32));
                    //  std::cout << "metadata.event_code OK!!!"  << std::endl;
                    continue;
                }
                async_msg_fifo.push_with_pop_on_full(metadata);

                pax::pax2::standard_async_msg_prints(metadata);
            }
            else{
                //TODO unknown received packet, may want to print error...
            }
        }catch(const std::exception &e){
            std::cerr<< "Error in recv pirate loop: " << e.what() << std::endl;
        }
    }
}

void net_work_init(mb_container_type& tester,pax::device_addrs_t& out,int N_STREAM){

    tester.iface = pax_iface::make(pax::transport::udp_simple::make_connected(
                                       out[0]["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
            ));
    tester.iface->poke32(U2_REG_SR_ADDR(U2_SR_SFC_PORT_REG), USRP2_UDP_FIFO_CRTL_PORT<<16);

    tester.fifo_ctrl = usrp2_fifo_ctrl::make(
                make_xport( out[0]["addr"], boost::lexical_cast<std::string>(USRP2_UDP_FIFO_CRTL_PORT), out[0], "recv")
            );

    tester.iface->set_fifo_ctrl(tester.fifo_ctrl);

    try{    //test fifo ctrl status
        boost::this_thread::sleep(boost::posix_time::milliseconds(3));
        tester.fifo_ctrl->peek32(SETTING_REGS_BASE);
        tester.spi_and_wb_iface = tester.fifo_ctrl;
        PAX_LOGGER_INFO("PAX")<<" fifo ctrl work fine";
    } catch(...){
        tester.spi_and_wb_iface = tester.iface;
        PAX_LOGGER_WARNING("PAX")<<"INFO: fifo ctrl dosen't work. normal zpu path replaced" ;
    }


    for(int i=0;i<N_STREAM;i++)
    {
        tester.rx_dsp_xports.push_back(make_xport(
                                           out[0]["addr"], boost::lexical_cast<std::string>(PAX8_UDP_RX_DSP_PORT(i)), out[0], "recv"
                ));
        tester.tx_dsp_xports.push_back(make_xport(
                                           out[0]["addr"], boost::lexical_cast<std::string>(PAX8_UDP_TX_DSP_PORT(i)), out[0], "send"
                ));

    }
    tester.wbiface = tester.iface;
    tester.spiface = tester.iface;
}

vec_streamers_t set_streams(mb_container_type& tester,pax::device_addrs_t& out,int N_STREAM){

    tester.rx_dsps.clear();
    for( int i=0;i<N_STREAM;i++)
    {
        tester.rx_dsps.push_back(rx_dsp_core_200::make(
                                     tester.wbiface, U2_REG_SR_ADDR(SR_RX_DSP(i)), U2_REG_SR_ADDR(SR_RX_CTRL(i)), USRP2_RX_SID_BASE + i, true
                                     ));
    }


    tester.tx_dsp.clear();
    for(unsigned int i=0;i<1;i++)
    {
        tester.tx_dsp.push_back(tx_dsp_core_200::make(
                                    tester.wbiface, U2_REG_SR_ADDR(SR_TX_DSP(i)), U2_REG_SR_ADDR(SR_TX_CTRL(i)), 0));

    }




    tester.rx_streamers.resize(N_STREAM);
    tester.tx_streamers.resize(1);
    pax::stream_args_t stream_arg;
    vec_streamers_t streamers;
    streamers.clear();

    //std::vector<boost::shared_ptr<io_impl> >  _io_impls(16,boost::shared_ptr<io_impl> (new io_impl)) ;

    boost::shared_ptr<io_impl> _io_impl;
    tester._io_impls.clear();

    for( int i=0;i<N_STREAM;i++)
    {
        boost::shared_ptr<pax::transport::sph::recv_packet_streamer> temp = get_rx_stream(stream_arg,tester,i);
        temp->set_xport_chan_get_buff(0, boost::bind(
                                          &pax::transport::zero_copy_if::get_recv_buff,tester.rx_dsp_xports[i], _1
                                          ));
        streamers.push_back(temp);
        
    }
    tester._io_impls.push_back(boost::shared_ptr<io_impl> (new io_impl)) ;
        _io_impl=tester._io_impls[0];
        _io_impl->tx_xports.clear();
        _io_impl->tx_xports.push_back(tester.tx_dsp_xports[0]);
        _io_impl->fc_mons.push_back(flow_control_monitor::sptr(new flow_control_monitor(
                                                                   out[0].cast("send_buff_size", 1<<16) /
                                                               tester.tx_dsp_xports[0]->get_send_frame_size()
                                                               )));

        //spawn a new pirate to plunder the recv booty
        _io_impl->pirate_tasks.push_back(pax::task::make(boost::bind(
                                                             &io_impl::recv_pirate_loop, _io_impl.get(),
                                                             tester.tx_dsp_xports[0], 0
                                                             )));

        pax::tx_streamer::sptr ss = get_tx_stream(stream_arg,_io_impl,tester,0);
    return streamers;
}

void init_time(mb_container_type& tester){

    time64_core_200::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_hi_now = U2_REG_TIME64_HI_RB_IMM;
    time64_rb_bases.rb_lo_now = U2_REG_TIME64_LO_RB_IMM;
    time64_rb_bases.rb_hi_pps = U2_REG_TIME64_HI_RB_PPS;
    time64_rb_bases.rb_lo_pps = U2_REG_TIME64_LO_RB_PPS;

    static const size_t mimo_clock_sync_delay_cycles = 138;
    tester.time64=time64_core_200::make(
                tester.spi_and_wb_iface, U2_REG_SR_ADDR(SR_TIME64), time64_rb_bases, mimo_clock_sync_delay_cycles
                );
    tester.time64->set_tick_rate(DSP_CLOCK_RATE);
}

void read_fw_and_init(mb_container_type& tester){
    PAX_LOGGER_INFO("PAX")<<"firmware version is: "<<tester.iface->get_fw_version_string()<<std::endl;
    const boost::uint32_t fpga_compat_num =tester.iface->peek32(U2_REG_COMPAT_NUM_RB);
    boost::uint16_t fpga_major = fpga_compat_num >> 16;
    pax::flash::flash_type_t flash_type = static_cast<pax::flash::flash_type_t>(tester.iface->peekfw(U2_FW_REG_FLASH_TYPE));
    tester.iface->set_flash_type(flash_type);
    pax::flash::flash_interface_t flash_interface =  static_cast<pax::flash::flash_interface_t>(tester.iface->peekfw(U2_FW_REG_FLASH_INTERFACE_TYPE));
    tester.iface->set_flash_interface(flash_interface);


    if (fpga_major != USRP2_FPGA_COMPAT_NUM){

    }


    tester.iface->lock_device(true);

}
std::string read_flash(mb_container_type& tester){

    std::string device_id_SerialNumber(28,0);
    switch (tester.iface->get_flash_type()){
    case pax::flash::BPI_28F00AP30:{
        pax::flash_vec_t A,B,AB;
        tester.iface->read_otp_flash(0,A,8,pax::flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_2);
        tester.iface->read_otp_flash(0,B,6,pax::flash::BPI_28F00AP30_8_WORD_OTP_REG_SEC_3);
        AB.reserve( A.size() + B.size() ); // preallocate memory
        AB.insert( AB.end(), A.begin(), A.end() );
        AB.insert( AB.end(), B.begin(), B.end() );
        for(int i=0;i<14;i++)
        {
            device_id_SerialNumber[2*i+1]=AB[i]>>8;
            device_id_SerialNumber[2*i]=AB[i]&0xff;
        }
    }break;
    case pax::flash::SPI_IS25LPxxx:{
        pax::flash_vec_t temp;
        tester.iface->read_otp_flash(0x0,temp,28,pax::flash::IS25LPxx_OTP_REG_0x000_TO_0x0ff);
        for(uint8_t i=0;i<temp.size();i++){
            device_id_SerialNumber[i]=temp[i];
        }
    }break;
    case pax::flash::BPI_S29GL01GS:{
        pax::flash_vec_t temp;
        tester.iface->read_otp_flash(0,temp,16,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);
        for(uint8_t i=0;i<14;i++)
        {
            device_id_SerialNumber[2*i+1]=temp[i]>>8;
            device_id_SerialNumber[2*i]=temp[i]&0xff;
        }
        std::string str_0xff; str_0xff.push_back(0xff);
        boost::replace_all(device_id_SerialNumber,str_0xff, " ");
    }
        break;
    default :throw pax::not_implemented_error("this flash type is not implemented");
    }
    return device_id_SerialNumber;
}
void set_board_type(mb_container_type& tester){

    std::string flash_str=read_flash(tester);

    flash_str[14]=' ';
    flash_str[15]=' ';

    tester.board = pax::usrp::NULL_D;
    if(!std::memcmp(flash_str.c_str(),"PAX2.D",6)){
        PAX_LOGGER_INFO("PAX")<<flash_str;
        tester.board=pax::usrp::pax2_D;
    }
    else if(!std::memcmp(flash_str.c_str(),"PAX8.D",6)){
        tester.board=pax::usrp::pax8_D_K325T;
        PAX_LOGGER_INFO("PAX")<<flash_str;
    }

    else if(!std::memcmp(flash_str.c_str(),"PAX2S6",6))
    {
        tester.board=pax::usrp::pax2s6_D;
        PAX_LOGGER_INFO("PAX")<<flash_str;
    }
    else if (!std::memcmp(flash_str.c_str(),"PAXGNS",6)){
        tester.board=pax::usrp::pax8_D_K325T;
        PAX_LOGGER_INFO("PAX")<<flash_str;
    }
    else if(!std::memcmp(flash_str.c_str(),"8K410T",6)){
        tester.board=pax::usrp::pax8_D_K410T;
        PAX_LOGGER_INFO("PAX")<<flash_str;
    }
    else if(!std::memcmp(flash_str.c_str(),"PAX8V7",6)){
        tester.board=pax::usrp::pax8v7_D;
        PAX_LOGGER_INFO("PAX")<<flash_str;
    }
    else
        PAX_LOGGER_WARNING("PAX")<<"Warning: Unknown carrier board!";

}


void chech_incompatible_state(mb_container_type& tester){
    switch (tester.daughter_b) {
    case pax::usrp::pax8 :{
        if(tester.board == pax::usrp::pax8_D_K325T)
            return;
        tester.board =  pax::usrp::pax8_D_K325T;
    } break;
    case pax::usrp::pax2_9361_filter_bank :
    case pax::usrp::pax2 :{
        if(tester.board == pax::usrp::pax2_D)
            return;
        tester.board =  pax::usrp::pax2_D;
    } break;
    case pax::usrp::pax2s6 :{
        if(tester.board == pax::usrp::pax2s6_D)
            return;
        tester.board =  pax::usrp::pax2s6_D;
    } break;
    case pax::usrp::pax8_gnss_4ch :
    case pax::usrp::pax8_gnss_8ch_calibrasion :{
        if(tester.board == pax::usrp::pax8_D_K325T)
            return;
        tester.board =  pax::usrp::pax8_D_K325T;
    } break;
    case pax::usrp::pax8_gnss_8ch :
    case pax::usrp::pax8_gnss_8ch_monitoring :{
        if(tester.board == pax::usrp::pax8_D_K410T)
            return;
        tester.board = pax::usrp::pax8_D_K410T;
    }break;
    case pax::usrp::pax8v7_9361_filter_bank :{
        if(tester.board == pax::usrp::pax8v7_D)
            return;
        tester.board = pax::usrp::pax8v7_D;
    }break;
    case pax::usrp::NULL_B:
        std::cout<<"cant recognize daughter try to guess the type of daughter"<<std::endl<<"WARNING: daughter set to ";
        switch(tester.board){
        case pax::usrp::pax2s6_D:
            std::cout<<"pax2 spartan 6"<<std::endl;
            tester.daughter_b = pax::usrp::pax2s6;
            break;
        case pax::usrp::pax2_D:
            std::cout<<"pax2 kintex 7 with filterbank"<<std::endl;
            tester.daughter_b = pax::usrp::pax2_9361_filter_bank;
            break;
        case pax::usrp::pax8_D_K325T:
            std::cout<<"pax8 kintex 7 325T with 4 AD9361"<<std::endl;
            tester.daughter_b = pax::usrp::pax8;
            break;
        case pax::usrp::pax8_D_K410T:
            std::cout<<"pax8 kintex 7 410T with 8 AD9361"<<std::endl;
            tester.daughter_b = pax::usrp::pax8_gnss_8ch_monitoring;
            break;
        case pax::usrp::pax8v7_D:
            std::cout<<"pax8 virtex 7 with 4 AD9361"<<std::endl;
            tester.daughter_b = pax::usrp::pax8v7_9361_filter_bank;
            break;
        case pax::usrp::NULL_D:
            tester.daughter_b = pax::usrp::pax2_9361_filter_bank;
            tester.board =  pax::usrp::pax2_D;
            std::cout<<"NO guess for this bouard matched"<<std::endl;
            break;
        }
        break;
    default: throw pax::not_implemented_error("this type of bouard is not implemented!!"); break;
    }
    std::cerr<<"this board is incompatible with this daughter board possible problem in flash (wrong otp set) or EEPROM"<<std::endl;

}

boost::uint32_t set_num_ad9361(mb_container_type& tester){
    tester.N_AD9361=0;
    tester.N_AD9364=0;
    switch(tester.daughter_b){
    case pax::usrp::Analog:
        tester.N_AD9361=1;
        break;
    case pax::usrp::pax8:
        tester.N_AD9361=4;
        break;
    case pax::usrp::pax2s6:
        tester.N_AD9364=2;
        break;
    case pax::usrp::pax8_gnss_4ch:
        tester.N_AD9364=4;
        break;
    case pax::usrp::pax8_gnss_8ch_monitoring:
    case pax::usrp::pax8_gnss_8ch:
    case pax::usrp::pax8_gnss_8ch_calibrasion:
        tester.N_AD9364=8;
        break;
    case pax::usrp::pax2:
    case pax::usrp::pax2_9361_filter_bank:
        tester.N_AD9361=1;
        break;
    case pax::usrp::pax8v7_9361_filter_bank:
        tester.N_AD9361=4;
        break;
    case pax::usrp::NULL_B:
        break;
    default :
        throw pax::not_implemented_error("this error is not implemented");
    }
    return  (tester.N_AD9361 + tester.N_AD9364);
}


uint32_t set_daughter_board(mb_container_type& tester){
    pax::eeprom::sptr db = pax::eeprom::make(tester.iface);
    std::string eeprom_value = db->info.product_name;

    std::string _ad_ref_clk  = db->info.ad_ref_clk;
    if(_ad_ref_clk == "20"){
        tester.ad_ref_clk = pax::usrp::_20MHz;
    }
    else{
        tester.ad_ref_clk = pax::usrp::_40MHz;
    }



    if(eeprom_value=="Analog")
    {
        tester.daughter_b=pax::usrp::Analog;
    }
    else if(eeprom_value=="PAX8 Analog Kit")
    {
        tester.daughter_b=pax::usrp::pax8;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX8 daughter board wit 4 AD9361s."<<std::endl;
    }
    else if(eeprom_value=="PAX2 Analog Kit")
    {
        tester.daughter_b=pax::usrp::pax2;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX2 daughter board with one AD9361."<<std::endl;
    }
    else if(eeprom_value=="PAX2S6 Analog Kit")
    {
        tester.daughter_b=pax::usrp::pax2s6;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX2S6 daughter board with 2 AD9364."<<std::endl;
    }
    else if(eeprom_value=="PAXGNS Analog Kit")
    {
        tester.daughter_b=pax::usrp::pax8_gnss_8ch;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX8K7 daughter board with 8 AD9364"<<std::endl;
    }
    else if(eeprom_value=="PAXGNS Analog CAL")
    {
        tester.daughter_b=pax::usrp::pax8_gnss_8ch_calibrasion;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX8K7 325T daughter board with 8 AD9364 with phase calibrasion"<<std::endl;
    }
    else if(eeprom_value=="PAXGNS8 Monitorin")
    {
        tester.daughter_b=pax::usrp::pax8_gnss_8ch_monitoring;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX8K7 410T daughter board with 8 AD9364 with monitoring"<<std::endl;
    }
    else if(eeprom_value=="PAXGNS4 AnalogKit")
    {
        tester.daughter_b=pax::usrp::pax8_gnss_4ch;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX8K7 daughter board with 4 AD9364"<<std::endl;
    }
    else if (eeprom_value=="PAX2K7 FILTERBANK")
    {
        tester.daughter_b=pax::usrp::pax2_9361_filter_bank;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX2K7 daughter board with 1 AD9361"<<std::endl;
    }
    else if (eeprom_value=="PAX8V7 FILTERBANK")
    {
        tester.daughter_b=pax::usrp::pax8v7_9361_filter_bank;
        PAX_LOGGER_INFO("PAX")<<"Detected PAX8V7 daughter board with 4 AD9361"<<std::endl;
    }
    else
    {
        tester.daughter_b=pax::usrp::NULL_B;
        PAX_LOGGER_INFO("PAX")<<"Daugther board doesn't exist."<<std::endl;
    }
    chech_incompatible_state(tester);
    return set_num_ad9361(tester);
}


void net_work_init(mb_container_type& tester,pax::device_addrs_t& out){

    tester.iface = pax_iface::make(pax::transport::udp_simple::make_connected(
                                       out[0]["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
            ));
    tester.iface->poke32(U2_REG_SR_ADDR(U2_SR_SFC_PORT_REG), USRP2_UDP_FIFO_CRTL_PORT<<16);
    tester.fifo_ctrl = usrp2_fifo_ctrl::make(
                make_xport( out[0]["addr"], boost::lexical_cast<std::string>(USRP2_UDP_FIFO_CRTL_PORT), out[0], "recv")
            );
    tester.iface->set_fifo_ctrl(tester.fifo_ctrl);
}
void init_ad9361(mb_container_type& tester,uint32_t N_AD9361){
    ////////////////////////////////////////////////////////////////
    // configure RF SPI and WB interface                         //
    ////////////////////////////////////////////////////////////////

    tester.iface->poke32(U2_REG_SR_ADDR(SR_SPI_CORE),50); //SET SPEED OF SPI DEVIDE TO 50
//    tester.iface->poke32(U2_REG_SR_ADDR(SR_ADC_CLK_EN), 0x0);


    ////////////////////////////////////////////////////////////////
    // INIT   A2D and RFIC  interface                         //
    ////////////////////////////////////////////////////////////////


    adf4351::sptr _adf4351;
    if (tester.daughter_b==pax::usrp::pax8){
        _adf4351=adf4351::make(tester.spiface,1<<4);
        _adf4351->set_refin_freq(10e6);
        //adf4351->set_freq(70e6);
        _adf4351->powerdown(true);
        // enable lo ext synth -> distributor link
        tester.iface->poke32(U2_REG_SR_ADDR(SR_LO_SEL), 0x7);
        tester.iface->poke32(U2_REG_SR_ADDR(SR_LO_EN), 0x1);
    }


    tester.ad_9361.clear();

    for(uint32_t i=0;i<N_AD9361;i++){
        pax::usrp::ad9361_params::sptr client_settings = boost::make_shared<pax::usrp::pax_ad9361_client_t>();
        client_settings->set_db(tester.daughter_b);
        client_settings->set_mb(tester.board);
        client_settings->set_interface(tester.iface);

        tester.ad_9361.push_back( pax::usrp::ad9361_ctrl::make_spi(client_settings,tester.spi_and_wb_iface, 1<<i, i, _adf4351, tester.ad_ref_clk));
    }
}


void board_specefic_initializing(mb_container_type& tester){
    tester.sync=pax::pax_sync::make(tester.ad_9361,tester.wbiface);
    switch(tester.board){
    case pax::usrp::pax2s6_D: {
        tester.iface->set_FPGA_devices(pax::XILINX_FPGA::XC6SLX150);
#ifdef __HAND_OFF__
        for (int i=0;i<2;i++){
            tester.filter_bank.push_back( pax::filter_bank_handoff::make(tester.ad_9361,tester.iface, tester.spi_and_wb_iface,i));
            tester.filter_bank[i]->filter_bank_init();
        }
#endif
    }break;
    case pax::usrp::pax8_D_K325T : {
        tester.iface->set_FPGA_devices(pax::XILINX_FPGA::XC7K325T);
        switch (tester.daughter_b){
        case pax::usrp::pax8_gnss_8ch_calibrasion: {
            tester.sync->do_mcs(); // MultiChipSync = mcs
            for(int i=0;i<8;i++)
                tester.iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<31)));// not to multiply in 0
            
            tester.sync->GNS_pass_channel();// open channel
        }break;
        case pax::usrp::pax8_gnss_8ch :{
            tester.sync->do_mcs();
            for(int i=0;i<8;i++)
                tester.iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<31)));

            tester.sync->GNS_calibration(1575.42e6 , 50e6 , true);
            for(uint8_t i=0;i<8;i++){
                tester.filter_bank.push_back( pax::filter_bank_nullptr::make(tester.ad_9361,tester.iface, tester.spi_and_wb_iface, i));
                tester.filter_bank[i]->filter_bank_init();
            }

        }break;
        case pax::usrp::pax8_gnss_4ch : {
            tester.sync->do_mcs();
            for(int i=0;i<8;i++)
                tester.iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<31)));
        }break;
        case pax::usrp::pax8:{
            tester.sync->PAX8K7_rx_cal_mode(false);// calibrasion mode
        } break;
        default:{throw pax::not_implemented_error("not implemented this type of pax8_D_K325T");}break;
        }
    } break;
    case pax::usrp::pax8_D_K410T : {
        tester.iface->set_FPGA_devices(pax::XILINX_FPGA::XC7K410T);
        switch (tester.daughter_b){
        case pax::usrp::pax8_gnss_8ch_monitoring : {
            tester.iface->poke32(U2_REG_SR_ADDR(812),0xffff);
            tester.sync->do_mcs();
            for(int i=0;i<8;i++)
            tester.iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<30)));
            for(uint8_t i=0;i<8;i++){
                tester.filter_bank.push_back( pax::filter_bank_simulator::make(tester.ad_9361,tester.iface, tester.spi_and_wb_iface,i));
                tester.filter_bank[i]->filter_bank_init();
            }
        }break;
        case pax::usrp::pax8_gnss_8ch: {
            tester.iface->poke32(U2_REG_SR_ADDR(812),0x0000);
            tester.sync->do_mcs();
            for(int i=0;i<8;i++)
            tester.iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<30)));
           tester.iface->poke32(U2_REG_GPIO_ADDR(1),0xff);
        }break;
        default:{throw pax::not_implemented_error("not implemented this type of pax8_D_K410T");}break;
        }
    }break;
    case pax::usrp::pax2_D : {
        tester.iface->set_FPGA_devices(pax::XILINX_FPGA::XC7K325T);
        tester.compass = pax::compass::make(tester.iface,3);
        tester.compass->run();
        tester.gps = pax::gps::make(tester.iface,2);
        tester.gps->run();
        switch(tester.daughter_b){
        case pax::usrp::pax2_9361_filter_bank : {
            tester.filter_bank.push_back(pax::filter_bank_aseman2::make(tester.ad_9361,tester.iface, tester.spi_and_wb_iface,0));
            tester.filter_bank[0]->filter_bank_init();
        } break;
        case pax::usrp::pax2: break;
        default : throw pax::not_implemented_error("this daughter board is not implemented");
        }
    }break;
    case pax::usrp::pax8v7_D :{
        tester.iface->set_FPGA_devices(pax::XILINX_FPGA::XC7V690T);
        tester.sync->do_mcs();
        for(uint8_t i=0;i<4;i++){
            tester.filter_bank.push_back( pax::filter_bank_virtex::make(tester.ad_9361,tester.iface, tester.spi_and_wb_iface,i));
            tester.filter_bank[i]->filter_bank_init();
        }
        for(uint8_t i=0;i<8;i++)
            tester.iface->poke32(U2_REG_SR_ADDR(SR_PHASE_DELAY_VALUE(i)),((1<<31)));
        tester.sync->PAX8V7_rx_cal_mode(false);// calibrasion mode

    }break;
    default : {
        std::cout<<"no spesefic configuration dedicated";
        tester.iface->set_FPGA_devices(pax::XILINX_FPGA::UNKOWN);
    }break;

    }

}
pax::device_addrs_t pax_find_init(pax::device_addr_t hint){
    pax::device_addrs_t out = usrp2_find(hint);

    BOOST_FOREACH(pax::device_addr_t& dev,out)
    {
        PAX_LOGGER_INFO("PAX")<<boost::format("Found PAX in %s and OK control!\n")%dev["addr"];
    }
    if(out.size()==0)
    {
        PAX_LOGGER_FATAL("PAX")<<boost::format("NO PAX FOUND")<<std::endl;
        throw std::invalid_argument( "Pax is not Connected" );
        //exit(0);
    }
    return out;
}


pax::device_addr_t pax_find( pax::eth_ip_addr_t ip){
    pax::device_addr_t hint;
    pax::device_addrs_t temp=usrp2_find(hint);
    for(uint32_t i=0;i<temp.size();i++){
        if(temp[i]["addr"] == ip.get_addr()){
            return temp[i];
        }
    }
    std::cerr<<"can not find this ip: "<<ip.get_addr()<<std::endl;
    return hint;
}
vec_IP_t find_all_pax_ip(){
    pax::device_addr_t hint;
    pax::device_addrs_t temp=usrp2_find(hint);
    static vec_IP_t return_values;
    for(uint32_t i =0;i<temp.size();i++){
        return_values.push_back(temp[i]["addr"]);
    }
    std::sort(return_values.begin(),return_values.end());
    return return_values;
}
pax_dev_t pax_init(size_t N_STREAM,pax::device_addr_t hint, bool bypass_board_specefic_init){
    mb_container_type tester;
    pax_dev_t out;
    out.streamers=pax_init(tester,N_STREAM,hint,bypass_board_specefic_init);
    out.dev=tester;
    out.pax_dev_addr= hint;
    return out;

}
pax_dev_t pax_init(size_t N_STREAM,std::string IP, bool bypass_board_specefic_init){
    pax::eth_ip_addr_t ip;
    ip.set_addr(IP);
    return pax_init(N_STREAM,pax_find(ip),bypass_board_specefic_init);
}

vec_streamers_t pax_init(mb_container_type& tester,size_t N_STREAM, bool bypass_board_specefic_init)
{
    const  pax::device_addr_t hint;
    return pax_init(tester,N_STREAM,hint,bypass_board_specefic_init);
}


vec_streamers_t pax_init(mb_container_type& tester,size_t N_STREAM,pax::device_addr_t hint, bool bypass_board_specefic_init)
{
    pax::device_addrs_t founded_devices = pax_find_init(hint);
    net_work_init(tester,founded_devices,N_STREAM);
    init_time(tester);
    read_fw_and_init(tester);
    set_board_type(tester);

    int N_AD936x=set_daughter_board(tester);
    vec_streamers_t streamers ;
    if(N_STREAM != 0){
        init_ad9361(tester,N_AD936x);
        streamers = set_streams( tester,founded_devices, N_STREAM);
    }
        if(!bypass_board_specefic_init){
            board_specefic_initializing(tester);//
            if(!tester.filter_bank.empty() && !tester.ad_9361.empty()){
                for(uint8_t k = 0; k < N_AD936x; k++)
                    tester.ad_9361[k]->set_filter_bank(tester.filter_bank[k]);
                tester.sync->set_filter_bank(tester.filter_bank);
            }
        }

    return streamers;
}




#include <static.hpp>

mb_container_type pax_radio;


