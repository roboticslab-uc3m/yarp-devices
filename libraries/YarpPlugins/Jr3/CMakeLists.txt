# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(Jr3
                    CATEGORY device
                    TYPE roboticslab::Jr3
                    INCLUDE Jr3.hpp
                    DEFAULT ON
                    DEPENDS UNIX
                    EXTRA_CONFIG WRAPPER=analogServer)

if(NOT SKIP_Jr3)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(Jr3 Jr3.hpp
                        jr3pci-ioctl.h
                        DeviceDriverImpl.cpp
                        IAnalogSensorImpl.cpp)

    target_link_libraries(Jr3 YARP::YARP_os
                              YARP::YARP_dev
                              ROBOTICSLAB::ColorDebug)

    yarp_install(TARGETS Jr3
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
