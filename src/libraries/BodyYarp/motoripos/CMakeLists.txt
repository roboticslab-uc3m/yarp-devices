# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

prepare_device(motoripos TYPE teo::MotorIpos INCLUDE MotorIpos.hpp WRAPPER controlboard)

IF (NOT SKIP_motoripos)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(motoripos MotorIpos.cpp IPositionControlRawImpl.cpp)
  TARGET_LINK_LIBRARIES(motoripos ${YARP_LIBRARIES})
  install(TARGETS motoripos DESTINATION lib)

  # Exporting dependencies for BODYConfig.cmake quite manually for now... 
  set(BODY_LIBRARIES ${BODY_LIBRARIES} BodyYarp motoripos CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_motoripos)

