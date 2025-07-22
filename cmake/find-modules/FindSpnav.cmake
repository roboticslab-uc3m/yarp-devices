# Find the Spnav library and header.
#
# Sets the following variables:
#
# Spnav_FOUND        - System has Spnav
# Spnav_INCLUDE_DIRS - Spnav include directories
# Spnav_LIBRARIES    - Spnav libraries
#
# ...and the following imported targets (requires CMake 2.8.11+):
#
# Spnav::Spnav       - the Spnav library

if(UNIX)
    find_package(PkgConfig)
    pkg_check_modules(PC_Spnav QUIET spnav)

    if(NOT Spnav_INCLUDE_DIR)
        find_path(Spnav_INCLUDE_DIR NAMES spnav.h
                                    PATHS ${PC_Spnav_INCLUDEDIR})
    endif()

    if(NOT Spnav_LIBRARY)
        find_library(Spnav_LIBRARY NAMES spnav libspnav
                                   PATHS ${PC_Spnav_LIBDIR})
    endif()

    if(PC_Spnav_FOUND)
        set(Spnav_VERSION ${PC_Spnav_VERSION})
    else()
        set(Spnav_VERSION 0.2)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Spnav REQUIRED_VARS Spnav_INCLUDE_DIR Spnav_LIBRARY
                                        VERSION_VAR Spnav_VERSION)

if(Spnav_FOUND)
    set(Spnav_INCLUDE_DIRS ${Spnav_INCLUDE_DIR})
    set(Spnav_LIBRARIES ${Spnav_LIBRARY})

    if(NOT TARGET Spnav::Spnav)
        add_library(Spnav::Spnav UNKNOWN IMPORTED)

        set_target_properties(Spnav::Spnav PROPERTIES IMPORTED_LOCATION "${Spnav_LIBRARY}"
                                                      INTERFACE_INCLUDE_DIRECTORIES "${Spnav_INCLUDE_DIR}")
    endif()
endif()

mark_as_advanced(Spnav_INCLUDE_DIR Spnav_LIBRARY)
