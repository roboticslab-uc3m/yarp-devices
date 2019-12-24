# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CuiAbsolute
                    CATEGORY device
                    TYPE roboticslab::CuiAbsolute
                    INCLUDE CuiAbsolute.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_CanBusSharerLib)

if(NOT SKIP_CuiAbsolute)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(CuiAbsolute CuiAbsolute.hpp
                                CuiAbsolute.cpp
                                DeviceDriverImpl.cpp
                                ICanBusSharerImpl.cpp
                                IEncodersRawImpl.cpp)

    target_link_libraries(CuiAbsolute YARP::YARP_OS
                                      YARP::YARP_dev
                                      ROBOTICSLAB::ColorDebug
                                      CanBusSharerLib
                                      YarpDevicesInterfaces)

    yarp_install(TARGETS CuiAbsolute
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
