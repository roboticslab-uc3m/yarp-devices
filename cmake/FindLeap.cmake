#.rst
# FindLeap
# ------------
#
# Created by Walter Gray.
# Locate and configure Leap
#
# Interface Targets
# ^^^^^^^^^^^^^^^^^
#   Leap::Leap
#
# Variables
# ^^^^^^^^^
#   Leap_ROOT_DIR
#   Leap_FOUND
#   Leap_INCLUDE_DIR
#   Leap_LIBRARY

set(_external_library_dir /usr
                          /usr/local
                          /opt
                          /opt/leap
                          /opt/leap_sdk
                          /opt/LeapSDK)

find_path(Leap_ROOT_DIR
          NAMES include/Leap.h
          HINTS ${_external_library_dir}
          PATH_SUFFIXES LeapSDK-${Leap_FIND_VERSION}
                        LeapSDK)

if(EXISTS "${Leap_ROOT_DIR}/version.txt")
    file(READ "${Leap_ROOT_DIR}/version.txt" _version_text)
    string(REGEX REPLACE "^([0-9]+\\.[0-9]+\\.[0-9]+).*" "\\1" Leap_VERSION "${_version_text}")

    if(Leap_VERSION STREQUAL "")
        message(WARNING "Unable to parse Leap SDK version file \"version.txt\".")
    endif()
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_arch_suffix lib/x64)
else()
    set(_arch_suffix lib/x86)
endif()

set(Leap_INCLUDE_DIR "${Leap_ROOT_DIR}/include")

if(NOT APPLE)
    find_library(Leap_LIBRARY "Leap" HINTS "${Leap_ROOT_DIR}/${_arch_suffix}")
else()
    string(FIND "${CMAKE_CXX_FLAGS}" "-stdlib=libc++" found_lib)

    if(${found_lib} GREATER -1)
        set(_libdir ${Leap_ROOT_DIR}/lib)
    else()
        set(_libdir ${Leap_ROOT_DIR}/lib/libstdc++)
    endif()

    find_library(Leap_LIBRARY
                 NAMES libLeap.dylib
                 HINTS "${_libdir}")
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Leap
                                  REQUIRED_VARS Leap_ROOT_DIR Leap_INCLUDE_DIR Leap_LIBRARY
                                  VERSION_VAR Leap_VERSION)
