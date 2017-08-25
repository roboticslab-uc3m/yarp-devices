# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(LeapMotionSensor
                    CATEGORY device
                    TYPE roboticslab::LeapMotionSensor
                    INCLUDE LeapMotionSensor.hpp
                    WRAPPER analogServer)

if(NOT SKIP_LeapMotionSensor)
  find_package(Leap 2.3 EXACT REQUIRED)

  include_directories(${YARP_INCLUDE_DIRS} ${Leap_INCLUDE_DIR} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(LeapMotionSensor
                  LeapMotionSensor.cpp
                  LeapMotionSensor.hpp
                  DeviceDriverImpl.cpp
                  IAnalogSensorImpl.cpp)

  target_link_libraries(LeapMotionSensor ${YARP_LIBRARIES} ${Leap_LIBRARY})

  yarp_install(TARGETS LeapMotionSensor
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES LeapMotionSensor.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
