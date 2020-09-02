add_compile_options(-I.)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    LIBPAX_APPEND_SOURCES(
        ${CMAKE_CURRENT_SOURCE_DIR}/synthesizer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ADF4351.cpp
    )

