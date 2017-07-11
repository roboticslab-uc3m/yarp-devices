# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & David Estevez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)

YARP_PREPARE_PLUGIN(AravisGigE CATEGORY device TYPE roboticslab::AravisGigE INCLUDE AravisGigE.hpp WRAPPER grabber)

IF (NOT SKIP_AravisGigE)
  find_package(ARAVIS REQUIRED)
  include_directories(${ARAVIS_INCLUDE_DIRS})

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(AravisGigE AravisGigE.hpp DeviceDriverImpl.cpp IFrameGrabberImpl.cpp IFrameGrabberImageRawImpl.cpp IFrameGrabberControls2Impl.cpp)
  TARGET_COMPILE_FEATURES(AravisGigE PRIVATE cxx_auto_type)
  TARGET_LINK_LIBRARIES(AravisGigE ${YARP_LIBRARIES} ${ARAVIS_LIBRARIES})

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now...
  set(ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
  #set(ROBOTICSLAB_YARP_DEVICES_LIBRARIES ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} LacqueyFetch CACHE INTERNAL "appended libraries")

  #install(TARGETS AravisGigE DESTINATION lib)
  yarp_install(TARGETS AravisGigE
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES AravisGigE.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_AravisGigE)


