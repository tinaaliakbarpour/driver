

########################################################################
SET(_pax_enabled_components "" CACHE INTERNAL "" FORCE)
SET(_pax_disabled_components "" CACHE INTERNAL "" FORCE)

########################################################################
# Register a component into the system
#  - name the component string name
#  - var the global enable variable
#  - enb the default enable setting
#  - deps a list of dependencies
#  - dis the default disable setting
########################################################################
MACRO(LIBPAX_REGISTER_COMPONENT name var enb deps dis)
    MESSAGE(STATUS "")
    MESSAGE(STATUS "Configuring ${name} support...")
    FOREACH(dep ${deps})
        MESSAGE(STATUS "  Dependency ${dep} = ${${dep}}")
    ENDFOREACH(dep)

    #setup the dependent option for this component
    INCLUDE(CMakeDependentOption)
    CMAKE_DEPENDENT_OPTION(${var} "enable ${name} support" ${enb} "${deps}" ${dis})

    #append the component into one of the lists
    IF(${var})
        MESSAGE(STATUS "  Enabling ${name} support.")
        LIST(APPEND _pax_enabled_components ${name})
    ELSE(${var})
        MESSAGE(STATUS "  Disabling ${name} support.")
        LIST(APPEND _pax_disabled_components ${name})
    ENDIF(${var})
    MESSAGE(STATUS "  Override with -D${var}=ON/OFF")

    #make components lists into global variables
    SET(_pax_enabled_components ${_pax_enabled_components} CACHE INTERNAL "" FORCE)
    SET(_pax_disabled_components ${_pax_disabled_components} CACHE INTERNAL "" FORCE)
ENDMACRO(LIBPAX_REGISTER_COMPONENT)

########################################################################
# Install only if appropriate for package and if component is enabled
########################################################################
FUNCTION(PAX_INSTALL)
    include(CMakeParseArgumentsCopy)
    CMAKE_PARSE_ARGUMENTS(PAX_INSTALL "" "DESTINATION;COMPONENT" "TARGETS;FILES;PROGRAMS" ${ARGN})

    IF(PAX_INSTALL_FILES)
        SET(TO_INSTALL "${PAX_INSTALL_FILES}")
    ELSEIF(PAX_INSTALL_PROGRAMS)
        SET(TO_INSTALL "${PAX_INSTALL_PROGRAMS}")
    ELSEIF(PAX_INSTALL_TARGETS)
        SET(TO_INSTALL "${PAX_INSTALL_TARGETS}")
    ENDIF(PAX_INSTALL_FILES)

    MESSAGE(khodaie TO_INSTALL value is "     " ${TO_INSTALL} ) 

    IF(PAX_INSTALL_COMPONENT STREQUAL "headers")
        IF(NOT LIBPAX_PKG AND NOT PAXHOST_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT PAXHOST_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "devel")
        IF(NOT LIBPAX_PKG AND NOT PAXHOST_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT PAXHOST_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "examples")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "tests")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "utilities")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "manual")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "doxygen")
        IF(NOT LIBPAX_PKG AND NOT PAXHOST_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT PAXHOST_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "manpages")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "images")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG AND NOT PAXHOST_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG AND NOT PAXHOST_PKG)
    ELSEIF(PAX_INSTALL_COMPONENT STREQUAL "readme")
        IF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG AND NOT PAXHOST_PKG)
            INSTALL(${ARGN})
        ENDIF(NOT LIBPAX_PKG AND NOT LIBPAXDEV_PKG AND NOT PAXHOST_PKG)
    ENDIF(PAX_INSTALL_COMPONENT STREQUAL "headers")
ENDFUNCTION(PAX_INSTALL)

########################################################################
# Print the registered component summary
########################################################################
FUNCTION(PAX_PRINT_COMPONENT_SUMMARY)
    MESSAGE(STATUS "")
    MESSAGE(STATUS "######################################################")
    MESSAGE(STATUS "# PAX enabled components                              ")
    MESSAGE(STATUS "######################################################")
    FOREACH(comp ${_pax_enabled_components})
        MESSAGE(STATUS "  * ${comp}")
    ENDFOREACH(comp)

    MESSAGE(STATUS "")
    MESSAGE(STATUS "######################################################")
    MESSAGE(STATUS "# PAX disabled components                             ")
    MESSAGE(STATUS "######################################################")
    FOREACH(comp ${_pax_disabled_components})
        MESSAGE(STATUS "  * ${comp}")
    ENDFOREACH(comp)

    MESSAGE(STATUS "")
ENDFUNCTION(PAX_PRINT_COMPONENT_SUMMARY)
