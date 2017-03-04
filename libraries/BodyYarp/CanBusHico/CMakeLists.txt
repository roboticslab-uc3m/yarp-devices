# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_PLUGIN(CanBusHico CATEGORY device TYPE teo::CanBusHico INCLUDE CanBusHico.hpp)

IF (NOT SKIP_CanBusHico)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(CanBusHico DeviceDriverImpl.cpp CanBusHico.cpp CanBusHico.cpp)

  TARGET_LINK_LIBRARIES(CanBusHico ${YARP_LIBRARIES})
  install(TARGETS CanBusHico DESTINATION lib)

  # Exporting dependencies for YARP_DEVICESConfig.cmake quite manually for now... 
#  set(YARP_DEVICES_INCLUDE_DIRS ${YARP_DEVICES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${COLOR_DEBUG_INCLUDE_DIRS} CACHE INTERNAL "appended libraries")
#  set(YARP_DEVICES_LIBRARIES ${YARP_DEVICES_LIBRARIES} CanBusHico CACHE INTERNAL "appended libraries") 

  #install(TARGETS CanBusHico DESTINATION lib)
  yarp_install(TARGETS CanBusHico
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES CanBusHico.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_CanBusHico)

