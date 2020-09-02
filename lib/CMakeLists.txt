
########################################################################
# Helpful Macros
########################################################################
MACRO(LIBPAX_APPEND_SOURCES)
    LIST(APPEND libpax_sources ${ARGV})
ENDMACRO(LIBPAX_APPEND_SOURCES)

MACRO(LIBPAX_APPEND_LIBS)
    LIST(APPEND libpax_libs ${ARGV})
ENDMACRO(LIBPAX_APPEND_LIBS)

MACRO(LIBPAX_PYTHON_GEN_SOURCE pyfile outfile)
    #ensure that the directory exists for outfile
    GET_FILENAME_COMPONENT(outfile_dir ${outfile} PATH)
    FILE(MAKE_DIRECTORY ${outfile_dir})

    #make the outfile depend on the python script
    ADD_CUSTOM_COMMAND(
        OUTPUT ${outfile} DEPENDS ${pyfile} ${LIBPAX_PYTHON_GEN_SOURCE_DEPS}
        COMMAND ${PYTHON_EXECUTABLE} -B ${pyfile} ${outfile}
        COMMENT "Generating ${outfile}"
    )

    #make libpax depend on the outfile
    LIBPAX_APPEND_SOURCES(${outfile})
ENDMACRO(LIBPAX_PYTHON_GEN_SOURCE)

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
INCLUDE_SUBDIRECTORY(AD9361)
INCLUDE_SUBDIRECTORY(type)
INCLUDE_SUBDIRECTORY(convert)
INCLUDE_SUBDIRECTORY(pax)
INCLUDE_SUBDIRECTORY(FLASH)
INCLUDE_SUBDIRECTORY(synthesizer)
INCLUDE_SUBDIRECTORY(synchronize)
INCLUDE_SUBDIRECTORY(eeprom)
INCLUDE_SUBDIRECTORY(filter_bank)
INCLUDE_SUBDIRECTORY(compass)
INCLUDE_SUBDIRECTORY(gps)
INCLUDE_SUBDIRECTORY(servo)
########################################################################
# Append to the list of sources for lib pax
########################################################################
LIBPAX_APPEND_SOURCES(
    ${CMAKE_CURRENT_SOURCE_DIR}/filter_bank/filter_bank_imple.cpp

)

########################################################################
# Add DLL resource file to Windows build
########################################################################
IF(MSVC)
    CONFIGURE_FILE(
        ${CMAKE_CURRENT_SOURCE_DIR}/pax.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/pax.rc
    @ONLY)

    LIST(APPEND libpax_sources ${CMAKE_CURRENT_BINARY_DIR}/pax.rc)
ENDIF(MSVC)

########################################################################
# Setup libpax library
########################################################################
ADD_LIBRARY(pax SHARED ${libpax_sources})
TARGET_LINK_LIBRARIES(pax ${Boost_LIBRARIES} ${libpax_libs})
SET_TARGET_PROPERTIES(pax PROPERTIES DEFINE_SYMBOL "PAX_DLL_EXPORTS")
IF(NOT LIBPAXDEV_PKG)
    SET_TARGET_PROPERTIES(pax PROPERTIES SOVERSION "${PAX_VERSION_MAJOR}")
    SET_TARGET_PROPERTIES(pax PROPERTIES VERSION "${PAX_VERSION_MAJOR}.${PAX_VERSION_MINOR}")
ENDIF(NOT LIBPAXDEV_PKG)
IF(DEFINED LIBPAX_OUTPUT_NAME)
    SET_TARGET_PROPERTIES(pax PROPERTIES OUTPUT_NAME ${LIBPAX_OUTPUT_NAME})
ENDIF(DEFINED LIBPAX_OUTPUT_NAME)

IF(NOT PAXHOST_PKG) #Syntax makes it unusable by PAX_INSTALL
    INSTALL(TARGETS pax
        LIBRARY DESTINATION ${LIBRARY_DIR} COMPONENT libraries # .so file
        ARCHIVE DESTINATION ${LIBRARY_DIR} COMPONENT libraries # .lib file
        RUNTIME DESTINATION ${RUNTIME_DIR} COMPONENT libraries # .dll file
    )
ENDIF(NOT PAXHOST_PKG)
