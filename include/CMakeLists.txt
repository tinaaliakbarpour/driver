PAX_INSTALL(FILES
ad9361_client.h       fp_compare_epsilon.ipp    sensors.hpp
ad9361_ctrl.hpp       fw_common.h               serial.hpp
ad9361_device.h       gps.hpp                   static.hpp
ADF4351_cfg.h         if_addrs.hpp              stream_cmd.hpp
ADF4351.h             math.hpp                  stream.hpp
atomic.hpp            metadata.hpp              super_recv_packet_handler.hpp
buffer_pool.hpp       micron.hpp                tasks.hpp
byteswap.hpp          time_spec.hpp		pax_reload.hpp
byteswap.ipp          msg_task.hpp              tx_dsp_core_200.hpp
CMakeLists.txt        pax_iface.h               udp_common.hpp
compass.hpp           pax_impl.hpp              udp_simple.hpp
config.h              pax_regs.hpp              udp_zero_copy.hpp
convert.hpp           usrp2_fifo_ctrl.hpp       super_send_packet_handler.hpp
device_addr.hpp       pax_sync.h                utility.hpp
dict.hpp              paxtype.h                 vrt_if_packet.hpp
dict.ipp              platform.hpp              wb_iface.hpp
eeprom.hpp            ranges.hpp                window.h
exception.hpp         ref_vector.hpp            rx_dsp_core_200.hpp
filters.hpp           rx_frontend_core_200.hpp  zero_copy.hpp
fp_compare_delta.ipp  safe_call.h               bounded_buffer.hpp
bounded_buffer.ipp     thread_priority.hpp      async_packet_handler.hpp
time64_core_200.hpp   filter_bank.hpp           fpga_device.hpp
spi_wb_iface.hpp       thread.hpp log.hpp       narrow.hpp
paths.hpp             isatty.hpp                version.hpp 
version_master.h       log_add.hpp              noncopyable.hpp
filter_bank_imple.hpp


  DESTINATION ${INCLUDE_DIR}/pax
  COMPONENT headers
)
