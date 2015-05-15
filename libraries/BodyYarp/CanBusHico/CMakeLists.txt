# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

prepare_device(CanBusHico TYPE teo::CanBusHico INCLUDE CanBusHico.hpp)

IF (NOT SKIP_CanBusHico)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${TEO_BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(CanBusHico DeviceDriverImpl.cpp CanBusHico.cpp CanBusHico.cpp)
  TARGET_LINK_LIBRARIES(CanBusHico ${YARP_LIBRARIES})
  install(TARGETS CanBusHico DESTINATION lib)

  # Exporting dependencies for TEO_BODYConfig.cmake quite manually for now... 
  set(TEO_BODY_INCLUDE_DIRS ${TEO_BODY_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${COLOR_DEBUG_INCLUDE_DIRS} CACHE INTERNAL "appended libraries") 
  set(TEO_BODY_LIBRARIES ${TEO_BODY_LIBRARIES} CanBusHico CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_CanBusHico)

