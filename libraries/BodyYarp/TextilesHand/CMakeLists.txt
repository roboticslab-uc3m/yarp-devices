# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_PLUGIN(TextilesHand CATEGORY device TYPE teo::TextilesHand INCLUDE TextilesHand.hpp)

IF (NOT SKIP_TextilesHand)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${TEO_BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

  yarp_add_plugin(TextilesHand TextilesHand.cpp DeviceDriverImpl.cpp IControlLimits2RawImpl.cpp IControlModeRawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControlRawImpl.cpp ICanBusSharerImpl.cpp )

  TARGET_LINK_LIBRARIES(TextilesHand ${YARP_LIBRARIES})
  install(TARGETS TextilesHand DESTINATION lib)

  # Exporting dependencies for TEO_BODYConfig.cmake quite manually for now... 
#  set(TEO_BODY_LIBRARIES ${TEO_BODY_LIBRARIES} TextilesHand CACHE INTERNAL "appended libraries") 

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

