# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(Jr3
                    CATEGORY device
                    TYPE roboticslab::Jr3
                    INCLUDE Jr3.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_Jr3)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(Jr3 Jr3.hpp
                    jr3pci-ioctl.h
                    DeviceDriverImpl.cpp
                    IAnalogSensorImpl.cpp)

add_dependencies(Jr3 COLOR_DEBUG)

target_link_libraries(Jr3 YARP::YARP_OS
                          YARP::YARP_dev
                          YarpDevicesInterfaces)

yarp_install(TARGETS Jr3
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES Jr3.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
