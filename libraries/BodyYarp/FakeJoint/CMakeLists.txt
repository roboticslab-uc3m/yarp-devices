# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_PLUGIN(FakeJoint CATEGORY device TYPE teo::FakeJoint INCLUDE FakeJoint.hpp)

IF (NOT SKIP_FakeJoint)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(FakeJoint FakeJoint.cpp DeviceDriverImpl.cpp IControlLimits2RawImpl.cpp IControlModeRawImpl.cpp IEncodersRawImpl.cpp IEncodersTimedRawImpl.cpp IPositionControl2RawImpl.cpp ICanBusSharerImpl.cpp )

  TARGET_LINK_LIBRARIES(FakeJoint ${YARP_LIBRARIES})
  install(TARGETS FakeJoint DESTINATION lib)

  # Exporting dependencies for ROBOTICSLAB_YARP_DEVICESConfig.cmake quite manually for now...
#  set(ROBOTICSLAB_YARP_DEVICES_LIBRARIES ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} FakeJoint CACHE INTERNAL "appended libraries") 

  #install(TARGETS FakeJoint DESTINATION lib)
  yarp_install(TARGETS FakeJoint
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES FakeJoint.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_FakeJoint)

