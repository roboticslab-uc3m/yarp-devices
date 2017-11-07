# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_PLUGIN(TextilesHand CATEGORY device TYPE roboticslab::TextilesHand INCLUDE TextilesHand.hpp)

IF (NOT SKIP_TextilesHand)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(TextilesHand TextilesHand.cpp DeviceDriverImpl.cpp IControlLimits2RawImpl.cpp IControlMode2RawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControl2RawImpl.cpp ICanBusSharerImpl.cpp IPositionDirectRawImpl.cpp ITorqueControlRawImpl.cpp IVelocityControl2RawImpl.cpp IInteractionModeRawImpl.cpp)

  TARGET_LINK_LIBRARIES(TextilesHand ${YARP_LIBRARIES})

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now...
#  set(ROBOTICSLAB_YARP_DEVICES_LIBRARIES ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} TextilesHand CACHE INTERNAL "appended libraries") 

  #install(TARGETS TextilesHand DESTINATION lib)
  yarp_install(TARGETS TextilesHand
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES TextilesHand.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

ENDIF (NOT SKIP_TextilesHand)

