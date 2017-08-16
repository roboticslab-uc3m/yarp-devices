# Find the xwiimote library and header.
#
# Sets the following variables:
#
# XWIIMOTE_FOUND        - System has xwiimote
# XWIIMOTE_INCLUDE_DIRS - xwiimote include directories
# XWIIMOTE_LIBRARIES    - xwiimote libraries

if(UNIX)
    if(NOT XWIIMOTE_INCLUDE_DIR)
        find_path(XWIIMOTE_INCLUDE_DIR xwiimote.h)
    endif()

    if(NOT XWIIMOTE_LIBRARY)
        find_library(XWIIMOTE_LIBRARY NAMES xwiimote libxwiimote)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XWiimote DEFAULT_MSG XWIIMOTE_INCLUDE_DIR XWIIMOTE_LIBRARY)

mark_as_advanced(XWIIMOTE_INCLUDE_DIR XWIIMOTE_LIBRARY)

set(XWIIMOTE_INCLUDE_DIRS ${XWIIMOTE_INCLUDE_DIR})
set(XWIIMOTE_LIBRARIES ${XWIIMOTE_LIBRARY})
