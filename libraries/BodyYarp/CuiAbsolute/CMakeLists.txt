# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_DEVICE(CuiAbsolute TYPE teo::CuiAbsolute INCLUDE CuiAbsolute.hpp)

IF (NOT SKIP_CuiAbsolute)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS}  ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(CuiAbsolute CuiAbsolute.cpp DeviceDriverImpl.cpp IControlLimits2RawImpl.cpp IControlModeRawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControlRawImpl.cpp ICanBusSharerImpl.cpp )

  TARGET_LINK_LIBRARIES(CuiAbsolute ${YARP_LIBRARIES})
  install(TARGETS CuiAbsolute DESTINATION lib)

  # Exporting dependencies for YARP_DEVICESConfig.cmake quite manually for now... 
  set(YARP_DEVICES_LIBRARIES ${YARP_DEVICES_LIBRARIES} CuiAbsolute CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_CuiAbsolute)

