# Find the Phidgets libraries and headers.
#
# Sets the following variables:
#
# Phidgets_FOUND        - system has Phidgets
# Phidgets_VERSION      - Phidgets version
# Phidgets_INCLUDE_DIRS - Phidgets include directories
# Phidgets_LIBRARIES    - Phidgets libraries
#
# ...and one of the following imported targets (requires CMake 2.8.11+):
#
# Phidgets::Phidget21   - the phidget21 library
# Phidgets::Phidget22   - the phidget22 library
#
# Supported Phidgets versions: 2.1, 2.2.

if(NOT Phidgets_FIND_VERSION_MAJOR OR NOT Phidgets_FIND_VERSION_MINOR)
    message(STATUS "No major/minor version set in FindPhidgets, using the \"21\" suffix (e.g. phidget21).")
    set(_phidgets_version_suffix 21)
else()
    set(_phidgets_version_suffix "${Phidgets_FIND_VERSION_MAJOR}${Phidgets_FIND_VERSION_MINOR}")
endif()

set(_phidgets_full_name phidget${_phidgets_version_suffix})

if(UNIX)
    find_package(PkgConfig)
    pkg_check_modules(PC_Phidgets QUIET lib${_phidgets_full_name})

    if(NOT Phidgets_INCLUDE_DIR)
        find_path(Phidgets_INCLUDE_DIR NAMES ${_phidgets_full_name}.h
                                       PATHS ${PC_Phidgets_INCLUDEDIR})
    endif()

    if(NOT Phidgets_LIBRARY)
        find_library(Phidgets_LIBRARY NAMES ${_phidgets_full_name}
                                      PATHS ${PC_Phidgets_LIBDIR})
    endif()

    set(Phidgets_VERSION ${PC_Phidgets_VERSION})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Phidgets REQUIRED_VARS Phidgets_INCLUDE_DIR Phidgets_LIBRARY
                                           VERSION_VAR Phidgets_VERSION)

if(Phidgets_FOUND)
    set(Phidgets_INCLUDE_DIRS ${Phidgets_INCLUDE_DIR})
    set(Phidgets_LIBRARIES ${Phidgets_LIBRARY})
    set(_target_name Phidget${_phidgets_version_suffix})

    if(NOT TARGET Phidgets::${_target_name})
        add_library(Phidgets::${_target_name} UNKNOWN IMPORTED)

        set_target_properties(Phidgets::${_target_name} PROPERTIES IMPORTED_LOCATION "${Phidgets_LIBRARY}"
                                                                   INTERFACE_INCLUDE_DIRECTORIES "${Phidgets_INCLUDE_DIR}")
    endif()
endif()

mark_as_advanced(Phidgets_INCLUDE_DIR Phidgets_LIBRARY)
