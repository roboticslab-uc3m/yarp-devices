# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_DEVICE(Jr3 TYPE teo::Jr3 INCLUDE Jr3.hpp WRAPPER analogServer)

IF (NOT SKIP_Jr3)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${TEO_BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(Jr3 DeviceDriverImpl.cpp IGenericSensorImpl.cpp )

  TARGET_LINK_LIBRARIES(Jr3 ${YARP_LIBRARIES})
  install(TARGETS Jr3 DESTINATION lib)

  # Exporting dependencies for TEO_BODYConfig.cmake quite manually for now... 
  set(TEO_BODY_LIBRARIES ${TEO_BODY_LIBRARIES} Jr3 CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_Jr3)

