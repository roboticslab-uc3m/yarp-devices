# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(SpaceNavigator
                    CATEGORY device
                    TYPE roboticslab::SpaceNavigator
                    INCLUDE SpaceNavigator.hpp
                    WRAPPER analogServer)

if(NOT SKIP_SpaceNavigator)
  include_directories(${YARP_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})
  yarp_add_plugin(SpaceNavigator SpaceNavigator.cpp SpaceNavigator.hpp DeviceDriverImpl.cpp IAnalogSensorImpl.cpp)
  target_link_libraries(SpaceNavigator ${YARP_LIBRARIES} spnav)
  install(TARGETS SpaceNavigator DESTINATION lib)

  yarp_install(TARGETS SpaceNavigator
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES SpaceNavigator.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

