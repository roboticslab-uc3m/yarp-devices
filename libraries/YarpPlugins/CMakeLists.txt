# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

IF (ENABLE_YarpPlugins)

  # option(ENABLE_launchYarpPlugins "Choose if you want to compile launchYarpPlugins" TRUE)

  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})  
  include(YarpPlugin)

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now... 
  # Note: Need ICanBusSharer.h, which will also need CanBusHico/*.hpp for now
  set(ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/CanBusHico CACHE INTERNAL "appended libraries") 
  # Note: BodyYarp bodybot works, but bodybot BodyYarp fails.
  #set(ROBOTICSLAB_YARP_DEVICES_LIBRARIES ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} BodyYarp CACHE INTERNAL "appended libraries") 
  
 #  yarp_begin_plugin_library(YarpPlugins)
    add_subdirectory(CuiAbsolute)
    add_subdirectory(TechnosoftIpos)
    add_subdirectory(LacqueyFetch)
    add_subdirectory(TextilesHand)
    add_subdirectory(FakeJoint)
    add_subdirectory(CanBusHico)
    add_subdirectory(CanBusControlboard)
    add_subdirectory(Jr3)
    add_subdirectory(SpaceNavigator)
    add_subdirectory(WiimoteSensor)
    add_subdirectory(LeapMotionSensor)
    add_subdirectory(ProximitySensorsClient)
  # yarp_end_plugin_library(YarpPlugins)

  #install(TARGETS YarpPlugins DESTINATION lib)

  #IF (ENABLE_launchYarpPlugins)

  #  YARP_ADD_PLUGIN_YARPDEV_EXECUTABLE(launchYarpPlugins YarpPlugins)
  #  install(TARGETS launchYarpPlugins DESTINATION bin)

  #ENDIF (ENABLE_launchYarpPlugins)

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now... 
  set(ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")

ENDIF (ENABLE_YarpPlugins)

