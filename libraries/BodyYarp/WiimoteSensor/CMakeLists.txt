# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(WiimoteSensor
                    CATEGORY device
                    TYPE roboticslab::WiimoteSensor
                    INCLUDE WiimoteSensor.hpp
                    WRAPPER analogServer)

if(NOT SKIP_WiimoteSensor)
  find_package(XWiimote REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS} ${XWIIMOTE_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(WiimoteSensor
                  WiimoteSensor.hpp
                  DeviceDriverImpl.cpp
                  IAnalogSensorImpl.cpp)

  target_link_libraries(WiimoteSensor ${YARP_LIBRARIES} ${XWIIMOTE_LIBRARIES})

  yarp_install(TARGETS WiimoteSensor
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES WiimoteSensor.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
