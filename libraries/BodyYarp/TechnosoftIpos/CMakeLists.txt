# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_PLUGIN(TechnosoftIpos CATEGORY device TYPE teo::TechnosoftIpos INCLUDE TechnosoftIpos.hpp)

IF (NOT SKIP_TechnosoftIpos)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${TEO_BODY_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(TechnosoftIpos TechnosoftIpos.cpp DeviceDriverImpl.cpp IControlLimits2RawImpl.cpp IControlModeRawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControlRawImpl.cpp IPositionControl2RawImpl.cpp IPositionDirectRawImpl.cpp ITorqueControlRawImpl.cpp IVelocityControlRawImpl.cpp IVelocityControl2RawImpl.cpp ICanBusSharerImpl.cpp IInteractionModeRawImpl.cpp )

  TARGET_LINK_LIBRARIES(TechnosoftIpos ${YARP_LIBRARIES} ${TEO_BODY_LIBRARIES})
  install(TARGETS TechnosoftIpos DESTINATION lib)

  # Exporting dependencies for TEO_BODYConfig.cmake quite manually for now... 
  set(TEO_BODY_INCLUDE_DIRS ${TEO_BODY_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
 # set(TEO_BODY_LIBRARIES ${TEO_BODY_LIBRARIES} TechnosoftIpos CACHE INTERNAL "appended libraries") 

  #install(TARGETS TechnosoftIpos DESTINATION lib)
  yarp_install(TARGETS TechnosoftIpos
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES TechnosoftIpos.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

ENDIF (NOT SKIP_TechnosoftIpos)

