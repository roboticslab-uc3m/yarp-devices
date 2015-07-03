# Copyright: Universidad Carlos III de Madrid (C) 2013;
# Authors: Juan G. Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Exploit new cmake 2.6 features (export). 

# reduce warning level with cmake 2.6
cmake_minimum_required(VERSION 2.6)
#cmake policies
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(TEO_BODY)

### options: cpp libraries
option(ENABLE_BodyYarp "Choose if you want to compile BodyYarp" TRUE)
option(ENABLE_OneCanBusOneWrapper "Choose if you want to compile OneCanBusOneWrapper" TRUE)
option(ENABLE_TwoCanBusThreeWrappers "Choose if you want to compile TwoCanBusThreeWrappers" TRUE)

### options: cpp programs
option(ENABLE_launchManipulation "Choose if you want to compile launchManipulation" TRUE)
option(ENABLE_launchLocomotion "Choose if you want to compile launchLocomotion" TRUE)

option(ENABLE_oneCanBusOneWrapper "Choose if you want to compile oneCanBusOneWrapper" TRUE)

option(ENABLE_recordManipulation "Choose if you want to compile recordManipulation" TRUE)
option(ENABLE_playbackManipulation "Choose if you want to compile playbackManipulation" TRUE)

option(ENABLE_recordLocomotion "Choose if you want to compile recordLocomotion" TRUE)
option(ENABLE_playbackLocomotion "Choose if you want to compile playbackLocomotion" TRUE)

option(ENABLE_playbackServer "Choose if you want to compile playbackServer" TRUE)

### options: force default
option(ENABLE_BodyYarp_CanBusHico "Enable/disable compilation of BodyYarp_CanBusHico" TRUE)
option(ENABLE_BodyYarp_TechnosoftIpos "Enable/disable compilation of BodyYarp_TechnosoftIpos" TRUE)
option(ENABLE_BodyYarp_LacqueyFetch "Enable/disable compilation of BodyYarp_LacqueyFetch" TRUE)
option(ENABLE_BodyYarp_FakeJoint "Enable/disable compilation of BodyYarp_FakeJoint" TRUE)
option(ENABLE_BodyYarp_CanBusControlboard "Enable/disable compilation of BodyYarp_CanBusControlboard" TRUE)

# ColorDebug options
option(ColorDebug_FULL_FILE "Choose if you want to compile with CD_FULL_FILE" FALSE)
if(ColorDebug_FULL_FILE)
    add_definitions(-DCD_FULL_FILE)
endif(ColorDebug_FULL_FILE)

option(ColorDebug_HIDE_ERROR "Choose if you want to compile with CD_HIDE_ERROR" FALSE)
if(ColorDebug_HIDE_ERROR)
    add_definitions(-DCD_HIDE_ERROR)
endif(ColorDebug_HIDE_ERROR)

option(ColorDebug_HIDE_WARNING "Choose if you want to compile with CD_HIDE_WARNING" FALSE)
if(ColorDebug_HIDE_WARNING)
    add_definitions(-DCD_HIDE_WARNING)
endif(ColorDebug_HIDE_WARNING)

option(ColorDebug_HIDE_SUCCESS "Choose if you want to compile with CD_HIDE_SUCCESS" FALSE)
if(ColorDebug_HIDE_SUCCESS)
    add_definitions(-DCD_HIDE_SUCCESS)
endif(ColorDebug_HIDE_SUCCESS)

option(ColorDebug_HIDE_INFO "Choose if you want to compile with CD_HIDE_INFO" FALSE)
if(ColorDebug_HIDE_INFO)
    add_definitions(-DCD_HIDE_INFO)
endif(ColorDebug_HIDE_INFO)

option(ColorDebug_HIDE_DEBUG "Choose if you want to compile with CD_HIDE_DEBUG" FALSE)
if(ColorDebug_HIDE_DEBUG)
    add_definitions(-DCD_HIDE_DEBUG)
endif(ColorDebug_HIDE_DEBUG)

if(MSVC)
  MESSAGE(STATUS "Running on windows")    
  set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
    "Choose the type of build, recommanded options are: Debug or Release")
endif(NOT CMAKE_BUILD_TYPE)
# Hide variable to MSVC users, since it is not needed
if (MSVC)
   mark_as_advanced(CMAKE_BUILD_TYPE)
endif(MSVC)

######################

### this makes everything go in $TEO_BODY_DIR/lib and $TEO_BODY_DIR/bin
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
message(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")
# this doesn't happen automatically for makefiles
make_directory(${LIBRARY_OUTPUT_PATH})
make_directory(${EXECUTABLE_OUTPUT_PATH})
# and let us clean their contents on a "make clean"
##set_directory_properties(PROPERTIES LIBRARY_OUTPUT_PATH ADDITIONAL_MAKE_CLEAN_FILES)
##set_directory_properties(PROPERTIES EXECUTABLE_OUTPUT_PATH ADDITIONAL_MAKE_CLEAN_FILES)
mark_as_advanced(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)
mark_as_advanced(CMAKE_BACKWARDS_COMPATIBILITY)
##########################################

# Pick up our cmake modules - they are all in the cmake subdirectory
set(TEO_BODY_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${TEO_BODY_MODULE_PATH})

#set_property(GLOBAL PROPERTY TEO_BODY_INCLUDE_DIRS)
#set_property(GLOBAL PROPERTY BODY_LINK_DIRS)
#set_property(GLOBAL PROPERTY BODY_LIBRARIES)
set(TEO_BODY_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(TEO_BODY_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(TEO_BODY_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)

# add main contents
add_subdirectory(share)
add_subdirectory(libraries)
add_subdirectory(programs)

# export our variables to a TEO_BODYConfig.cmake creation
set(TEO_BODY_LINK_DIRS ${TEO_BODY_LINK_DIRS} ${LIBRARY_OUTPUT_PATH})
configure_file(${CMAKE_SOURCE_DIR}/cmake/template/TEO_BODYConfig.cmake.in
               ${CMAKE_BINARY_DIR}/TEO_BODYConfig.cmake @ONLY)

# create a symbolic link to the shared directory (as contains models and may be heavy)
# If you want a deep copy, you can use the copy_directory command instead of create_symlink.
#add_custom_target(install_applications COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/share ${CMAKE_SOURCE_DIR}/../share)

#endif(MSVC)
##add_custom_target(nuke
##  "${CMAKE_COMMAND}" -E "remove_directory" "${CMAKE_SOURCE_DIR}/bin" ";"
##  "${CMAKE_COMMAND}" -E "remove_directory" "${CMAKE_SOURCE_DIR}/lib")

#MESSAGE("\nDon't forget to set: " ${CMAKE_BINARY_DIR} " as your \${TEO-BODY_DIR} !!!\n")

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/template/TEO_BODYConfigUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/TEO_BODYConfigUninstall.cmake" IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/TEO_BODYConfigUninstall.cmake)

