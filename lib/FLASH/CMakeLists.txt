
MACRO(INCLUDE_SUBDIRECTORY subdir)
    #insert the current directories on the front of the list
    LIST(INSERT _cmake_source_dirs 0 ${CMAKE_CURRENT_SOURCE_DIR})
    LIST(INSERT _cmake_binary_dirs 0 ${CMAKE_CURRENT_BINARY_DIR})

    #set the current directories to the names of the subdirs
    SET(CMAKE_CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
    SET(CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/${subdir})

    #include the subdirectory CMakeLists to run it
    FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)

    #reset the value of the current directories
    LIST(GET _cmake_source_dirs 0 CMAKE_CURRENT_SOURCE_DIR)
    LIST(GET _cmake_binary_dirs 0 CMAKE_CURRENT_BINARY_DIR)

    #pop the subdir names of the front of the list
    LIST(REMOVE_AT _cmake_source_dirs 0)
    LIST(REMOVE_AT _cmake_binary_dirs 0)
ENDMACRO(INCLUDE_SUBDIRECTORY)

########################################################################
# Include subdirectories (different than add)
########################################################################
INCLUDE_SUBDIRECTORY(bpi_micron)
INCLUDE_SUBDIRECTORY(SPI_flash_IS25LPxxx)
INCLUDE_SUBDIRECTORY(BPI_flash_S29GL01GS)


add_compile_options(-I.)
include_directories()
    LIBPAX_APPEND_SOURCES(
        ${CMAKE_CURRENT_SOURCE_DIR}/pax_reload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pax_reload_impl.cpp
    )
