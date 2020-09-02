
LIBPAX_REGISTER_COMPONENT("PAX" ENABLE_PAX8 ON "ENABLE_LIBPAX" OFF)

    LIBPAX_APPEND_SOURCES(
        ${CMAKE_CURRENT_SOURCE_DIR}/ad9361_ctrl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ad9361_device.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/filters.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors.cpp
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ../synthesizer)
