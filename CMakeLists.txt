# Copyright: Universidad Carlos III de Madrid (C) 2013;
# Authors: Juan G. Victores & Raúl de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(ROBOTICSLAB_YARP_DEVICES)

# Force building shared libraries and YARP DL modules.
set(BUILD_SHARED_LIBS TRUE CACHE INTERNAL "hide this!")

# Add suffix for debug libraries.
if(MSVC)
    message(STATUS "Running on windows")
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
    # Possible values.
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    if(NOT CMAKE_BUILD_TYPE)
        # Encourage the user to specify build type.
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

# Pick up our CMake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules
                              ${CMAKE_SOURCE_DIR}/cmake/ycm-modules)

# Bootstrap YCM. Keep it compatible with cmake_minimum_required().
set(YCM_TAG v0.6.0)
include(YCMBootstrap)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Enabling coverage.
option(ENABLE_coverage "Choose if you want to enable coverage collection" OFF)

if(ENABLE_coverage)
    # List supported compilers.
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    endif()
endif()

# Superbuild phase - include color-debug.
include(FindOrBuildPackage)
find_or_build_package(COLOR_DEBUG)

# find_or_build_package() doesn't bring into scope COLOR_DEBUG_INCLUDE_DIRS
# nor other config variables. If a system copy is found, we have to call the
# find_package() command and proceed as usual.
if(USE_SYSTEM_COLOR_DEBUG)
    find_package(COLOR_DEBUG REQUIRED)
    include_directories(${COLOR_DEBUG_INCLUDE_DIRS})
    add_library(COLOR_DEBUG UNKNOWN IMPORTED)
endif()

# Store list of exported targets in global scope.
set_property(GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_TARGETS)

# Create targets if specific requirements are satisfied.
include(CMakeDependentOption)

# Find YARP (main dependency).
find_package(YARP REQUIRED)

# Load YARP modules.
if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
    include(YarpPlugin)
    include(YarpInstallationHelpers)
endif()

# Configure installation paths for YARP resources.
yarp_configure_external_installation(roboticslab-yarp-devices WITH_PLUGINS)

# Add main contents.
add_subdirectory(cmake)
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(tests)
add_subdirectory(share)

# Store the package in the user registry.
export(PACKAGE ROBOTICSLAB_YARP_DEVICES)

# Create and install config files.
include(InstallBasicPackageFiles)

# Set exported variables.
set(ROBOTICSLAB_YARP_DEVICES_BUILD_MODULE_DIR ${CMAKE_SOURCE_DIR}/cmake)
set(ROBOTICSLAB_YARP_DEVICES_INSTALL_MODULE_DIR ${CMAKE_INSTALL_DATADIR}/ROBOTICSLAB_YARP_DEVICES/cmake)

install_basic_package_files(ROBOTICSLAB_YARP_DEVICES
                            VERSION 0.1.0
                            COMPATIBILITY AnyNewerVersion
                            TARGETS_PROPERTY ROBOTICSLAB_YARP_DEVICES_TARGETS
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            NAMESPACE ROBOTICSLAB::
                            EXTRA_PATH_VARS_SUFFIX MODULE_DIR
                            CONFIG_TEMPLATE ${CMAKE_SOURCE_DIR}/cmake/templates/ROBOTICSLAB_YARP_DEVICESConfig.cmake.in
                            NO_COMPATIBILITY_VARS)

# Configure and create uninstall target.
include(AddUninstallTarget)
