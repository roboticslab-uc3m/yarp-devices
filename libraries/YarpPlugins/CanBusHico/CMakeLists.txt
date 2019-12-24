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

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

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
                                     ROBOTICSLAB::ColorDebug)

    yarp_install(TARGETS CanBusHico
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
