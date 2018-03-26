# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusFake
                    CATEGORY device
                    TYPE roboticslab::CanBusFake
                    INCLUDE CanBusFake.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_CanBusFake)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(CanBusFake CanBusFake.cpp
                           CanBusFake.hpp
                           DeviceDriverImpl.cpp)

add_dependencies(CanBusFake COLOR_DEBUG)

target_link_libraries(CanBusFake YARP::YARP_OS
                                 YARP::YARP_dev
                                 YarpDevicesInterfaces)

yarp_install(TARGETS CanBusFake
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CanBusFake.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
