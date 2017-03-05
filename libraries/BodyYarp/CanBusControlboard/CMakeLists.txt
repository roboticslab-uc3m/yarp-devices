# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_DEVICE(CanBusControlboard TYPE teo::CanBusControlboard INCLUDE CanBusControlboard.hpp WRAPPER controlboardwrapper2)

IF (NOT SKIP_CanBusControlboard)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(CanBusControlboard DeviceDriverImpl.cpp IControlLimits2Impl.cpp IControlModeImpl.cpp IEncodersImpl.cpp IEncodersTimedImpl.cpp IPositionControlImpl.cpp IPositionDirectImpl.cpp ITorqueControlImpl.cpp IVelocityControlImpl.cpp CanBusControlboard.cpp ThreadImpl.cpp)
  TARGET_LINK_LIBRARIES(CanBusControlboard ${YARP_LIBRARIES})
  install(TARGETS CanBusControlboard DESTINATION lib)

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now... 
  set(ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} CACHE INTERNAL "appended libraries") 
  set(ROBOTICSLAB_YARP_DEVICES_LIBRARIES ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} CanBusControlboard CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_CanBusControlboard)

