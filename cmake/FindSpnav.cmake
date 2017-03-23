# Find the Spnav library and header.
#
# Sets the following variables:
#
# SPNAV_FOUND        - System has Spnav
# SPNAV_INCLUDE_DIRS - Spnav include directories
# SPNAV_LIBRARIES    - Spnav libraries

if(UNIX)
    find_path(SPNAV_INCLUDE_DIR spnav.h)
    find_library(SPNAV_LIBRARY NAMES spnav libspnav)

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Spnav DEFAULT_MSG SPNAV_INCLUDE_DIR SPNAV_LIBRARY)

    mark_as_advanced(SPNAV_INCLUDE_DIR SPNAV_LIBRARY)

    set(SPNAV_INCLUDE_DIRS ${SPNAV_INCLUDE_DIR})
    set(SPNAV_LIBRARIES ${SPNAV_LIBRARY})
endif()
