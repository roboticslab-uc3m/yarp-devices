# Find the XWiimote library and header.
#
# Sets the following variables:
#
# XWiimote_FOUND        - system has XWiimote
# XWiimote_VERSION      - XWiimote version
# XWiimote_INCLUDE_DIRS - XWiimote include directories
# XWiimote_LIBRARIES    - XWiimote libraries

if(UNIX)
    find_package(PkgConfig)
    pkg_check_modules(XWiimote QUIET libxwiimote)

    if(NOT XWiimote_INCLUDE_DIR)
        find_path(XWiimote_INCLUDE_DIR xwiimote.h HINTS ${XWiimote_INCLUDEDIR})
    endif()

    if(NOT XWiimote_LIBRARY)
        find_library(XWiimote_LIBRARY NAMES xwiimote
                                      HINTS ${XWiimote_LIBDIR})
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XWiimote REQUIRED_VARS XWiimote_INCLUDE_DIR XWiimote_LIBRARY
                                           VERSION_VAR XWiimote_VERSION)

if(XWiimote_FOUND)
    set(XWiimote_INCLUDE_DIRS ${XWiimote_INCLUDE_DIR})
    set(XWiimote_LIBRARIES ${XWiimote_LIBRARY})
endif()

mark_as_advanced(XWiimote_INCLUDE_DIR XWiimote_LIBRARY)
