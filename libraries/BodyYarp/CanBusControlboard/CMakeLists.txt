# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

prepare_device(CanBusControlboard TYPE teo::CanBusControlboard INCLUDE CanBusControlboard.hpp WRAPPER controlboardwrapper2)

IF (NOT SKIP_CanBusControlboard)

  option(ENABLE_testCanBusControlboard "Choose if you want to compile testCanBusControlboard" TRUE)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(CanBusControlboard DeviceDriverImpl.cpp IControlLimitsImpl.cpp IControlModeImpl.cpp IEncodersImpl.cpp IEncodersTimedImpl.cpp IPositionControlImpl.cpp IPositionDirectImpl.cpp ITorqueControlImpl.cpp IVelocityControlImpl.cpp CanBusHico.cpp CanBusControlboard.cpp ThreadImpl.cpp)
  TARGET_LINK_LIBRARIES(CanBusControlboard ${YARP_LIBRARIES})
  install(TARGETS CanBusControlboard DESTINATION lib)

  # Exporting dependencies for TEO_BODYConfig.cmake quite manually for now... 
  set(TEO_BODY_INCLUDE_DIRS ${TEO_BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} CACHE INTERNAL "appended libraries") 
  set(TEO_BODY_LIBRARIES ${TEO_BODY_LIBRARIES} CanBusControlboard CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_CanBusControlboard)

