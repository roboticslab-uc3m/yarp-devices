# Find the XWiimote library and header.
#
# Sets the following variables:
#
# XWiimote_FOUND        - system has XWiimote
# XWiimote_VERSION      - XWiimote version
# XWiimote_INCLUDE_DIRS - XWiimote include directories
# XWiimote_LIBRARIES    - XWiimote libraries
#
# ...and the following imported targets (requires CMake 2.8.11+):
#
# XWiimote::XWiimote    - the XWiimote library

if(UNIX)
    find_package(PkgConfig)
    pkg_check_modules(XWiimote QUIET libxwiimote)

    if(NOT XWiimote_INCLUDE_DIR)
        find_path(XWiimote_INCLUDE_DIR NAMES xwiimote.h
                                       PATHS ${PC_XWiimote_INCLUDEDIR})
    endif()

    if(NOT XWiimote_LIBRARY)
        find_library(XWiimote_LIBRARY NAMES xwiimote
                                      PATHS ${PC_XWiimote_LIBDIR})
    endif()

    set(XWiimote_VERSION ${PC_Xwiimote_VERSION})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XWiimote REQUIRED_VARS XWiimote_INCLUDE_DIR XWiimote_LIBRARY
                                           VERSION_VAR XWiimote_VERSION)

if(XWiimote_FOUND)
    set(XWiimote_INCLUDE_DIRS ${XWiimote_INCLUDE_DIR})
    set(XWiimote_LIBRARIES ${XWiimote_LIBRARY})

    if(NOT CMAKE_VERSION VERSION_LESS 2.8.11 AND NOT TARGET XWiimote::XWiimote)
        add_library(XWiimote::XWiimote UNKNOWN IMPORTED)

        set_target_properties(XWiimote::XWiimote PROPERTIES IMPORTED_LOCATION "${XWiimote_LIBRARY}"
                                                            INTERFACE_INCLUDE_DIRECTORIES "${XWiimote_INCLUDE_DIR}")
    endif()
endif()

mark_as_advanced(XWiimote_INCLUDE_DIR XWiimote_LIBRARY)
