# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(ProximitySensorsClient
                    CATEGORY device
                    TYPE roboticslab::ProximitySensorsClient
                    INCLUDE ProximitySensorsClient.hpp)

if(NOT SKIP_ProximitySensors)

include_directories(${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

yarp_add_plugin(ProximitySensorsClient ProximitySensorsClient.hpp
                                       ProximitySensorsClient.cpp
                                       DeviceDriverImpl.cpp
                                       IProximitySensorsImpl.cpp)

target_link_libraries(ProximitySensorsClient ${YARP_LIBRARIES})

yarp_install(TARGETS ProximitySensorsClient
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES ProximitySensorsClient.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()

