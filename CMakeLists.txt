CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(PAX CXX C)
ENABLE_TESTING()





#make sure our local CMake Modules path comes first
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake/Modules)
######################################################################
#PROJECT SET
######################################################################
SET(HAND_OFF false)
SET(CFI false)



IF(HAND_OFF)
    ADD_DEFINITIONS(-D__HAND_OFF__)
ENDIF()
IF(CFI)
    ADD_DEFINITIONS(-D__CFI__)
ENDIF()



########################################################################
# Packaging Variables
########################################################################

SET(LIBPAX_PKG ${LIBPAX_PKG} CACHE BOOL "Build Debian libpax003 package")
SET(LIBPAXDEV_PKG ${LIBPAXDEV_PKG} CACHE BOOL "Build Debian libpax-dev package")
SET(PAXHOST_PKG ${PAXPOST_PKG} CACHE BOOL "Build Debian pax-host package")

INCLUDE(PAXComponent) #enable components
INCLUDE(PAXPackage)   #setup cpack

########################################################################
# Install Dirs
########################################################################
#when the library suffix should be 64 (applies to redhat linux family)
IF(NOT DEFINED LIB_SUFFIX AND REDHAT AND CMAKE_SYSTEM_PROCESSOR MATCHES "64$")
    SET(LIB_SUFFIX 64)
ENDIF()
SET(LIB_SUFFIX ${LIB_SUFFIX} CACHE STRING "lib directory suffix")
SET(RUNTIME_DIR bin)
SET(LIBRARY_DIR lib${LIB_SUFFIX})
SET(INCLUDE_DIR include)
SET(PKG_DATA_DIR share/pax)
IF(NOT DEFINED PKG_LIB_DIR)
    SET(PKG_LIB_DIR ${LIBRARY_DIR}/pax)
ENDIF()
SET(PKG_DOC_DIR share/doc/pax)
SET(PKG_MAN_DIR share/man/man1)

########################################################################
# Local Include Dir
########################################################################
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)


ADD_DEFINITIONS(-DPAX_LOG_MIN_LEVEL=info)
########################################################################
# GLOBAL MACRO
########################################################################

ADD_DEFINITIONS(-D__STDC_LIMIT_MACROS)

########################################################################
# On Apple only, set install name and use rpath correctly, if not already set
########################################################################

########################################################################
# On Apple only, set install name and use rpath correctly, if not already set
########################################################################
IF(APPLE)
    IF(NOT CMAKE_INSTALL_NAME_DIR)
        SET(CMAKE_INSTALL_NAME_DIR
            ${CMAKE_INSTALL_PREFIX}/${GR_LIBRARY_DIR} CACHE
            PATH "Library Install Name Destination Directory" FORCE)
    ENDIF(NOT CMAKE_INSTALL_NAME_DIR)
    IF(NOT CMAKE_INSTALL_RPATH)
        SET(CMAKE_INSTALL_RPATH
            ${CMAKE_INSTALL_PREFIX}/${GR_LIBRARY_DIR} CACHE
            PATH "Library Install RPath" FORCE)
    ENDIF(NOT CMAKE_INSTALL_RPATH)
    IF(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
        SET(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE
            BOOL "Do Build Using Library Install RPath" FORCE)
    ENDIF(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
ENDIF(APPLE)

########################################################################
# Optional Compiler Flags
########################################################################
INCLUDE(CheckCXXCompilerFlag)
MACRO(PAX_ADD_OPTIONAL_CXX_COMPILER_FLAG flag have)
    CHECK_CXX_COMPILER_FLAG(${flag} ${have})
    IF(${have})
        ADD_DEFINITIONS(${flag})
    ENDIF(${have})
ENDMACRO(PAX_ADD_OPTIONAL_CXX_COMPILER_FLAG)

#select the release build type by default to get optimization flags
IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "Debug")
   MESSAGE(STATUS "Build type not specified: defaulting to release.")
ENDIF(NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING "")

#force PAX_RELEASE_MODE to be a string for cmake-gui
SET(PAX_RELEASE_MODE "${PAX_RELEASE_MODE}" CACHE STRING "PAX Release Mode")

IF(CMAKE_COMPILER_IS_GNUCXX)
    IF(STRIP_BINARIES)
        IF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s")
        ENDIF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    ENDIF(STRIP_BINARIES)
    ADD_DEFINITIONS(-Wall)
    ADD_DEFINITIONS(-Wextra)
    ADD_DEFINITIONS(-Wsign-compare)
    #ADD_DEFINITIONS(-Wconversion)
    #ADD_DEFINITIONS(-pedantic)
    #ADD_DEFINITIONS(-ansi)


    IF(NOT WIN32)
        #only export symbols that are declared to be part of the pax api (non dll platforms)
        PAX_ADD_OPTIONAL_CXX_COMPILER_FLAG(-fvisibility=hidden HAVE_VISIBILITY_HIDDEN)
        PAX_ADD_OPTIONAL_CXX_COMPILER_FLAG(-fvisibility-inlines-hidden HAVE_VISIBILITY_INLINES_HIDDEN)
    ENDIF(NOT WIN32)


ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF(MSVC)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/cmake/msvc)
    ADD_DEFINITIONS( #stop all kinds of compatibility warnings
        -D_SCL_SECURE_NO_WARNINGS
        -D_SCL_SECURE_NO_DEPRECATE
        -D_CRT_SECURE_NO_WARNINGS
        -D_CRT_SECURE_NO_DEPRECATE
        -D_CRT_NONSTDC_NO_WARNINGS
        -D_CRT_NONSTDC_NO_DEPRECATE
		)

    ADD_DEFINITIONS(/MP) #multi-threaded build
ENDIF(MSVC)

IF(CYGWIN)
    ADD_DEFINITIONS(-D__USE_W32_SOCKETS) #boost asio says we need this
ENDIF(CYGWIN)

IF(WIN32)
    ADD_DEFINITIONS(-D_WIN32_WINNT=0x0501) #minimum version required is windows xp
    ADD_DEFINITIONS(-DNOMINMAX) #disables stupidity and enables std::min and std::max
    ADD_DEFINITIONS(-DWINDOWS) #disables stupidity and enables std::min and std::max
ENDIF(WIN32)

########################################################################
# Setup Boost
########################################################################
MESSAGE(STATUS "")
MESSAGE(STATUS "Configuring Boost C++ Libraries...")
SET(BOOST_REQUIRED_COMPONENTS
    date_time
    filesystem
    program_options
    regex
    system
    thread
    unit_test_framework
    serialization
    timer
    chrono
)

IF(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    LIST(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
ENDIF(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

IF(MSVC)
    SET(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    IF(BOOST_ALL_DYN_LINK)
        ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    ELSE(BOOST_ALL_DYN_LINK)
        SET(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    ENDIF(BOOST_ALL_DYN_LINK)
ENDIF(MSVC)

SET(Boost_ADDITIONAL_VERSIONS
    "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.48.0" "1.49" "1.50.0" "1.50"
    "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54" "1.55.0" "1.55"
    "1.56.0" "1.56" "1.57.0" "1.57" "1.62.0"
)
FIND_PACKAGE(Boost COMPONENTS ${BOOST_REQUIRED_COMPONENTS} REQUIRED)


INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

MESSAGE(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
MESSAGE(STATUS "Boost libraries: ${Boost_LIBRARIES}")

########################################################################
# Check Python Modules
########################################################################
INCLUDE(PAXPython)

PYTHON_CHECK_MODULE(
    "Python version 2.6 or greater"
    "platform" "platform.python_version() >= '2.6'"
    HAVE_PYTHON_PLAT_MIN_VERSION
)

# PYTHON_CHECK_MODULE(
#     "Cheetah templates 2.0.0 or greater"
#     "Cheetah" "Cheetah.Version >= '2.0.0'"
#     HAVE_PYTHON_MODULE_CHEETAH
# )

########################################################################
# Create Uninstall Target
########################################################################
CONFIGURE_FILE(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

ADD_CUSTOM_TARGET(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)



########################################################################
# Images download directory for utils/pax_images_downloader.py
########################################################################
#{{{IMG_SECTION
# This section is written automatically by /images/create_imgs_package.py
# Any manual changes in here will be overwritten.
#SET(PAX_IMAGES_MD5SUM "b20992748b90e6bdd228fe693f4d2ba2")
#SET(PAX_IMAGES_DOWNLOAD_SRC "pax-images_003.008.001-release.zip")
#}}}

########################################################################
# Register top level components
########################################################################
# LIBPAX_REGISTER_COMPONENT("LibPAX" ENABLE_LIBPAX ON "Boost_FOUND;HAVE_PYTHON_PLAT_MIN_VERSION;HAVE_PYTHON_MODULE_CHEETAH" OFF)
LIBPAX_REGISTER_COMPONENT("LibPAX" ENABLE_LIBPAX ON "Boost_FOUND;HAVE_PYTHON_PLAT_MIN_VERSION;" OFF)
LIBPAX_REGISTER_COMPONENT("Examples" ENABLE_EXAMPLES ON "ENABLE_LIBPAX" OFF)
LIBPAX_REGISTER_COMPONENT("Utils" ENABLE_UTILS ON "ENABLE_LIBPAX" OFF)
LIBPAX_REGISTER_COMPONENT("Tests" ENABLE_TESTS ON "ENABLE_LIBPAX" OFF)

########################################################################
# Add the subdirectories
########################################################################

IF(ENABLE_LIBPAX)
    ADD_SUBDIRECTORY(lib)
ENDIF(ENABLE_LIBPAX)

ADD_SUBDIRECTORY(include)

ADD_SUBDIRECTORY(bin)
ADD_SUBDIRECTORY(Hex2bin-2.4)


########################################################################
# Create Pkg Config File
########################################################################
FOREACH(inc ${Boost_INCLUDE_DIRS})
    LIST(APPEND PAX_PC_CFLAGS "-I${inc}")
ENDFOREACH(inc)

FOREACH(lib ${Boost_LIBRARY_DIRS})
    LIST(APPEND PAX_PC_LIBS "-L${lib}")
ENDFOREACH(lib)


#use space-separation format for the pc file
STRING(REPLACE ";" " " PAX_PC_REQUIRES "${PAX_PC_REQUIRES}")
STRING(REPLACE ";" " " PAX_PC_CFLAGS "${PAX_PC_CFLAGS}")
STRING(REPLACE ";" " " PAX_PC_LIBS "${PAX_PC_LIBS}")

#unset these vars to avoid hard-coded paths to cross environment
IF(CMAKE_CROSSCOMPILING)
    SET(PAX_PC_CFLAGS)
    SET(PAX_PC_LIBS)
ENDIF(CMAKE_CROSSCOMPILING)

#CONFIGURE_FILE(
 #   ${CMAKE_CURRENT_SOURCE_DIR}/pax.pc.in
  #  ${CMAKE_CURRENT_BINARY_DIR}/pax.pc
#@ONLY)

#PAX_INSTALL(
 #   FILES ${CMAKE_CURRENT_BINARY_DIR}/pax.pc
  #  DESTINATION ${LIBRARY_DIR}/pkgconfig
   # COMPONENT "devel"
#)

########################################################################
# Create and Install PAX cmake files
########################################################################
IF(NOT CMAKE_MODULES_DIR)
   SET(CMAKE_MODULES_DIR lib${LIB_SUFFIX}/cmake)
ENDIF(NOT CMAKE_MODULES_DIR)

CONFIGURE_FILE(
    ${CMAKE_SOURCE_DIR}/cmake/Modules/PAXConfigVersion.cmake.in
    ${CMAKE_BINARY_DIR}/cmake/Modules/PAXConfigVersion.cmake
    @ONLY
)

SET(cmake_configs
    ${CMAKE_SOURCE_DIR}/cmake/Modules/PAXConfig.cmake
    ${CMAKE_BINARY_DIR}/cmake/Modules/PAXConfigVersion.cmake
)

PAX_INSTALL(
    FILES ${cmake_configs}
    DESTINATION ${CMAKE_MODULES_DIR}/pax
    COMPONENT "devel"
)

########################################################################
# Handle pre-built images
########################################################################
IF(DEFINED PAX_IMAGES_DIR AND EXISTS "${PAX_IMAGES_DIR}")
    FILE(GLOB_RECURSE _image_files "${PAX_IMAGES_DIR}/*")
    MESSAGE(STATUS "Using images:")
    FOREACH(_img ${_image_files})
        MESSAGE(STATUS "  ${_img}")
    ENDFOREACH(_img)
    PAX_INSTALL(FILES ${_image_files} DESTINATION ${PKG_DATA_DIR}/images COMPONENT images)
ENDIF(DEFINED PAX_IMAGES_DIR AND EXISTS "${PAX_IMAGES_DIR}")

########################################################################
# Print Summary
########################################################################
IF(LIBPAX_PKG)
    MESSAGE(STATUS "")
    SET(PRINT_APPEND " (Debian libpax003 package configuration)")
ELSEIF(LIBPAXDEV_PKG)
    MESSAGE(STATUS "")
    SET(PRINT_APPEND " (Debian libpax-dev package configuration)")
ELSEIF(PAXHOST_PKG)
    MESSAGE(STATUS "")
    SET(PRINT_APPEND " (Debian pax-host package configuration)")
ENDIF(LIBPAX_PKG)
PAX_PRINT_COMPONENT_SUMMARY()
MESSAGE(STATUS "Building version: ${PAX_VERSION}${PRINT_APPEND}")
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
