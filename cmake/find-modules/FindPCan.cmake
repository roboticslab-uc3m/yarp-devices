# Find the PCan libraries and headers.
#
# Sets the following variables:
#
# PCan_FOUND        - system has PCan
# PCan_INCLUDE_DIRS - PCan include directories
# PCan_LIBRARIES    - PCan libraries
#
# ...and the following imported targets (requires CMake 2.8.11+):
#
# PCan::PCan        - PCan library (old API)
# PCan::PCanFD      - PCan library (v8+ API)

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

    if(NOT CMAKE_VERSION VERSION_LESS 2.8.11)
        if(NOT TARGET PCan::PCan)
            add_library(PCan::PCan UNKNOWN IMPORTED)

            set_target_properties(PCan::PCan PROPERTIES IMPORTED_LOCATION "${PCan_LIBRARY}"
                                                        INTERFACE_INCLUDE_DIRECTORIES "${PCan_INCLUDE_DIR}")
        endif()

	    if(NOT TARGET PCan::PCanFD)
            add_library(PCan::PCanFD UNKNOWN IMPORTED)

            set_target_properties(PCan::PCanFD PROPERTIES IMPORTED_LOCATION "${PCan_LIBRARY_FD}"
                                                          INTERFACE_INCLUDE_DIRECTORIES "${PCan_INCLUDE_DIR}")
        endif()
    endif()
endif()

mark_as_advanced(PCan_INCLUDE_DIR PCan_LIBRARY PCan_LIBRARY_FD)
