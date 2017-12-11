# Find the xwiimote library and header.
#
# Sets the following variables:
#
# XWIIMOTE_FOUND        - System has xwiimote
# XWIIMOTE_VERSION      - xwiimote version
# XWIIMOTE_INCLUDE_DIRS - xwiimote include directories
# XWIIMOTE_LIBRARIES    - xwiimote libraries

if(UNIX)
    find_package(PkgConfig)
    pkg_check_modules(XWIIMOTE QUIET libxwiimote)

    if(NOT XWIIMOTE_INCLUDE_DIR)
        find_path(XWIIMOTE_INCLUDE_DIR xwiimote.h HINTS ${XWIIMOTE_INCLUDEDIR})
    endif()

    if(NOT XWIIMOTE_LIBRARY)
        find_library(XWIIMOTE_LIBRARY NAMES xwiimote
                                      HINTS ${XWIIMOTE_LIBDIR})
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XWiimote REQUIRED_VARS XWIIMOTE_INCLUDE_DIR XWIIMOTE_LIBRARY
                                           VERSION_VAR XWIIMOTE_VERSION)

mark_as_advanced(XWIIMOTE_INCLUDE_DIR XWIIMOTE_LIBRARY)

set(XWIIMOTE_INCLUDE_DIRS ${XWIIMOTE_INCLUDE_DIR})
set(XWIIMOTE_LIBRARIES ${XWIIMOTE_LIBRARY})
