cmake_minimum_required(VERSION 3.19...3.28)

project(ROBOTICSLAB_YARP_DEVICES LANGUAGES CXX)

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
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.11 REQUIRED COMPONENTS os dev sig)

# Soft dependencies.
find_package(ARAVIS 0.4 QUIET)
find_package(Doxygen QUIET)
find_package(GTestSources 1.8 QUIET)
find_package(Leap 2.3 EXACT QUIET)
find_package(PCan 8.5 EXACT QUIET)
find_package(Phidgets 2.1 EXACT QUIET)
find_package(Python QUIET)
find_package(Spnav QUIET)
find_package(XWiimote 0.3 QUIET)

# Always build YARP devices as MODULE libraries.
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "Force dynamic plugins")

# Configure installation paths for YARP resources.
yarp_configure_external_installation(roboticslab-yarp-devices WITH_PLUGINS)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Create targets if specific requirements are satisfied.
include(CMakeDependentOption)

# Acknowledge this is a CTest-friendly project.
enable_testing()

# Add main contents.
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(tests)
add_subdirectory(share)
add_subdirectory(doc)
add_subdirectory(examples/cpp)

# Store the package in the user registry.
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

# Create and install config files.
include(InstallBasicPackageFiles)

install_basic_package_files(ROBOTICSLAB_YARP_DEVICES
                            VERSION 0.1.0
                            COMPATIBILITY AnyNewerVersion
                            NO_SET_AND_CHECK_MACRO
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            NAMESPACE ROBOTICSLAB::)

# Configure and create uninstall target.
include(AddUninstallTarget)
