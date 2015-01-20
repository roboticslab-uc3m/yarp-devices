# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

prepare_device(motorlacquey TYPE teo::MotorLacquey INCLUDE MotorLacquey.hpp)

IF (NOT SKIP_motorlacquey)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(motorlacquey MotorLacquey.cpp DeviceDriverImpl.cpp IControlLimitsRawImpl.cpp IControlModeRawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControlRawImpl.cpp IPositionDirectRawImpl.cpp ITorqueControlRawImpl.cpp IVelocityControlRawImpl.cpp ICanBusSharerImpl.cpp )

  TARGET_LINK_LIBRARIES(motorlacquey ${YARP_LIBRARIES})
  install(TARGETS motorlacquey DESTINATION lib)

  # Exporting dependencies for BODYConfig.cmake quite manually for now... 
  set(BODY_LIBRARIES ${BODY_LIBRARIES} BodyYarp motorlacquey CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_motorlacquey)

