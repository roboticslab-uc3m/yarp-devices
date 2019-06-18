# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusHico
                    CATEGORY device
                    TYPE roboticslab::CanBusHico
                    INCLUDE CanBusHico.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_CanBusHico)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(CanBusHico CanBusHico.cpp
                               CanBusHico.hpp
                               hico_api.h
                               DeviceDriverImpl.cpp
                               ICanBusImpl.cpp
                               HicoCanMessage.cpp
                               HicoCanMessage.hpp
                               FilterManager.cpp)

    target_link_libraries(CanBusHico YARP::YARP_OS
                                     YARP::YARP_dev
                                     ROBOTICSLAB::ColorDebug
                                     YarpDevicesInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS CanBusHico
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS CanBusHico
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES CanBusHico.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
