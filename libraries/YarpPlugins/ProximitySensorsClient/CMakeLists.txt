# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(ProximitySensorsClient
                    CATEGORY device
                    TYPE roboticslab::ProximitySensorsClient
                    INCLUDE ProximitySensorsClient.hpp
                    DEFAULT ON)

if(NOT SKIP_ProximitySensorsClient)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${Leap_INCLUDE_DIR})

yarp_add_plugin(ProximitySensorsClient ProximitySensorsClient.hpp
                                       ProximitySensorsClient.cpp
                                       DeviceDriverImpl.cpp
                                       IProximitySensorsImpl.cpp)

target_link_libraries(ProximitySensorsClient YARP::YARP_OS
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             YarpDevicesInterfaces)

yarp_install(TARGETS ProximitySensorsClient
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES ProximitySensorsClient.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
