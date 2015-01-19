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

project(TEO-BODY)

### options: cpp libraries
option(ENABLE_BodyYarp "Choose if you want to compile BodyYarp" TRUE)

### options: cpp modules
option(ENABLE_launchManipulation "Choose if you want to compile launchManipulation" FALSE)
option(ENABLE_recordManipulation "Choose if you want to compile recordManipulation" FALSE)
option(ENABLE_playbackManipulation "Choose if you want to compile playbackManipulation" FALSE)

option(ENABLE_launchLocomotion "Choose if you want to compile launchLocomotion" FALSE)
option(ENABLE_recordLocomotion "Choose if you want to compile recordLocomotion" FALSE)
option(ENABLE_playbackLocomotion "Choose if you want to compile playbackLocomotion" FALSE)

option(ENABLE_recordLeftManipulation "Choose if you want to compile recordLeftManipulation" FALSE)
option(ENABLE_playbackLeftManipulation "Choose if you want to compile playbackLeftManipulation" FALSE)

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

### this makes everything go in $BODY_DIR/lib and $BODY_DIR/bin
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
set(BODY_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${BODY_MODULE_PATH})

#set_property(GLOBAL PROPERTY BODY_INCLUDE_DIRS)
#set_property(GLOBAL PROPERTY BODY_LINK_DIRS)
#set_property(GLOBAL PROPERTY BODY_LIBRARIES)
set(BODY_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(BODY_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(BODY_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)

# add main contents
add_subdirectory(share)
add_subdirectory(src)

# export our variables to a BODYConfig.cmake creation
set(BODY_LINK_DIRS ${BODY_LINK_DIRS} ${LIBRARY_OUTPUT_PATH})
configure_file(${CMAKE_SOURCE_DIR}/cmake/template/BODYConfig.cmake.in
               ${CMAKE_BINARY_DIR}/BODYConfig.cmake @ONLY)

# create a symbolic link to the shared directory (as contains models and may be heavy)
# If you want a deep copy, you can use the copy_directory command instead of create_symlink.
#add_custom_target(install_applications COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/share ${CMAKE_SOURCE_DIR}/../share)

#endif(MSVC)
##add_custom_target(nuke
##  "${CMAKE_COMMAND}" -E "remove_directory" "${CMAKE_SOURCE_DIR}/bin" ";"
##  "${CMAKE_COMMAND}" -E "remove_directory" "${CMAKE_SOURCE_DIR}/lib")

#MESSAGE("\nDon't forget to set: " ${CMAKE_BINARY_DIR} " as your \${BODY_DIR} !!!\n")

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/template/BODYConfigUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/BODYConfigUninstall.cmake" IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/BODYConfigUninstall.cmake)

