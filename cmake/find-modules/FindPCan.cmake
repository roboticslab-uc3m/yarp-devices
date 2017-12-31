# Find the PCan libraries and headers.
#
# Sets the following variables:
#
# PCAN_FOUND        - system has PCan
# PCan_INCLUDE_DIRS - PCan include directories
# PCan_LIBRARIES    - PCan libraries

if(UNIX)
    find_path(PCan_INCLUDE_DIR NAMES pcan.h
                                     pcanfd.h
                                     libpcan.h
                                     libpcanfd.h)

    find_library(PCan_LIBRARY pcan)
    find_library(PCan_LIBRARY_FD pcanfd)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCan DEFAULT_MSG PCan_INCLUDE_DIR
                                                   PCan_LIBRARY
                                                   PCan_LIBRARY_FD)

if(PCan_FOUND)
    set(PCan_INCLUDE_DIRS ${PCan_INCLUDE_DIR})
    set(PCan_LIBRARIES ${PCan_LIBRARY} ${PCan_LIBRARY_FD})
endif()

mark_as_advanced(PCan_INCLUDE_DIR PCan_LIBRARY PCan_LIBRARY_FD)
