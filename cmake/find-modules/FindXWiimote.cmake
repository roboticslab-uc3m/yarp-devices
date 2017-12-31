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

if(XWiimote_FOUND AND NOT TARGET XWiimote::XWiimote)
    add_library(XWiimote::XWiimote UNKNOWN IMPORTED)

    set_target_properties(XWiimote::XWiimote PROPERTIES IMPORTED_LOCATION "${XWiimote_LIBRARY}"
                                                        INTERFACE_INCLUDE_DIRECTORIES "${XWiimote_INCLUDE_DIR}")
endif()

mark_as_advanced(XWiimote_INCLUDE_DIR XWiimote_LIBRARY)
