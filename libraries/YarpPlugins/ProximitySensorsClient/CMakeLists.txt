# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(ProximitySensorsClient
                    CATEGORY device
                    TYPE roboticslab::ProximitySensorsClient
                    INCLUDE ProximitySensorsClient.hpp
                    DEFAULT ON)

if(NOT SKIP_ProximitySensorsClient)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(ProximitySensorsClient ProximitySensorsClient.hpp
                                           ProximitySensorsClient.cpp
                                           DeviceDriverImpl.cpp
                                           IProximitySensorsImpl.cpp)

    target_link_libraries(ProximitySensorsClient YARP::YARP_os
                                                 YARP::YARP_dev
                                                 ROBOTICSLAB::YarpDevicesInterfaces)

    yarp_install(TARGETS ProximitySensorsClient
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
