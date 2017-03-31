# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_PLUGIN(LacqueyFetch CATEGORY device TYPE teo::LacqueyFetch INCLUDE LacqueyFetch.hpp)

IF (NOT SKIP_LacqueyFetch)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(LacqueyFetch LacqueyFetch.cpp DeviceDriverImpl.cpp IControlLimits2RawImpl.cpp IControlModeRawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControlRawImpl.cpp ICanBusSharerImpl.cpp )

  TARGET_LINK_LIBRARIES(LacqueyFetch ${YARP_LIBRARIES})

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now...
  set(ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
  #set(ROBOTICSLAB_YARP_DEVICES_LIBRARIES ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} LacqueyFetch CACHE INTERNAL "appended libraries")

  #install(TARGETS LacqueyFetch DESTINATION lib)
  yarp_install(TARGETS LacqueyFetch
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES LacqueyFetch.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

ENDIF (NOT SKIP_LacqueyFetch)

