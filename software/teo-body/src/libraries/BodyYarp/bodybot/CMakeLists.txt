# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

prepare_device(bodybot TYPE teo::BodyBot INCLUDE BodyBot.hpp WRAPPER controlboard)

IF (NOT SKIP_bodybot)

  option(ENABLE_testBodyBot "Choose if you want to compile testBodyBot" FALSE)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(bodybot DeviceDriverImpl.cpp IControlLimitsImpl.cpp IControlModeImpl.cpp IEncodersImpl.cpp IEncodersTimedImpl.cpp IPositionControlImpl.cpp IVelocityControlImpl.cpp ITorqueControlImpl.cpp CanBusHico.cpp MotorIpos.cpp BodyBot.cpp IPositionDirectImpl.cpp ThreadImpl.cpp IPositionControlRawImpl.cpp)
  TARGET_LINK_LIBRARIES(bodybot ${YARP_LIBRARIES})
  install(TARGETS bodybot DESTINATION lib)

  # Exporting dependencies for BODYConfig.cmake quite manually for now... 
  set(BODY_LIBRARIES ${BODY_LIBRARIES} BodyYarp bodybot CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_bodybot)

