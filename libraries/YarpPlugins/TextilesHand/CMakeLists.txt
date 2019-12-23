# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TextilesHand
                    CATEGORY device
                    TYPE roboticslab::TextilesHand
                    INCLUDE TextilesHand.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;ENABLE_CanBusSharerLib"
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_TextilesHand)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(TextilesHand TextilesHand.cpp
                                 TextilesHand.hpp
                                 DeviceDriverImpl.cpp
                                 IControlModeImpl.cpp
                                 IEncodersImpl.cpp
                                 IPositionControlImpl.cpp
                                 IPositionDirectImpl.cpp)

    target_link_libraries(TextilesHand YARP::YARP_OS
                                       YARP::YARP_dev
                                       ROBOTICSLAB::ColorDebug
                                       CanBusSharerLib)

    yarp_install(TARGETS TextilesHand
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
