

add_executable(pax_hex2bin hex2bin.c common.c libcrc.c binary.c)

PAX_INSTALL(TARGETS pax_hex2bin RUNTIME DESTINATION ${RUNTIME_DIR} COMPONENT utilities)
