# Find the PCan libraries and headers.
#
# Sets the following variables:
#
# PCan_FOUND        - system has PCan
# PCan_INCLUDE_DIRS - PCan include directories
# PCan_LIBRARIES    - PCan libraries
# PCan_VERSION      - PCan version (if supported)
#
# ...and the following imported targets (requires CMake 2.8.11+):
#
# PCan::PCan        - PCan library (old API)
# PCan::PCanFD      - PCan library (v8+ API)
#
# Supported versions: v8.5(.1).
# Hints: PCan_ROOT

if(UNIX)
    find_path(PCan_INCLUDE_DIR NAMES pcan.h
                                     pcanfd.h
                                     libpcan.h
                                     libpcanfd.h
                               HINTS $ENV{PCan_ROOT}
                               PATH_SUFFIXES include)

    find_library(PCan_LIBRARY NAMES pcan
                              HINTS $ENV{PCan_ROOT}
                              PATH_SUFFIXES lib)

    find_library(PCan_LIBRARY_FD NAMES pcanfd
                                 HINTS $ENV{PCan_ROOT}
                                 PATH_SUFFIXES lib)
endif()

set(PCan_VERSION PCan_VERSION-NOT_FOUND)

if(PCan_INCLUDE_DIR AND PCan_LIBRARY_FD)
    set(_cmake_include_dirs ${CMAKE_REQUIRED_INCLUDES})
    set(_cmake_libraries ${CMAKE_REQUIRED_LIBRARIES})

    list(APPEND CMAKE_REQUIRED_INCLUDES ${PCan_INCLUDE_DIR})
    list(APPEND CMAKE_REQUIRED_LIBRARIES ${PCan_LIBRARY_FD})

    include(CheckSymbolExists)

    # needs <sys/time.h>, probably an upstream bug in the API header
    check_symbol_exists(pcanfd_get_option "sys/time.h;libpcanfd.h" _pcan_compatible_8_5_1)

    if(_pcan_compatible_8_5_1)
        set(PCan_VERSION 8.5.1)
    else()
        message(STATUS "FindPCan.cmake reports unhandled PCan version (<8.5.1)")
    endif()

    set(CMAKE_REQUIRED_INCLUDES "${_cmake_include_dirs}")
    set(CMAKE_REQUIRED_LIBRARIES "${_cmake_libraries}")
    unset(_cmake_include_dirs)
    unset(_cmake_libraries)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCan FOUND_VAR PCan_FOUND
                                       REQUIRED_VARS PCan_INCLUDE_DIR
                                                     PCan_LIBRARY
                                                     PCan_LIBRARY_FD
                                       VERSION_VAR PCan_VERSION)

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
