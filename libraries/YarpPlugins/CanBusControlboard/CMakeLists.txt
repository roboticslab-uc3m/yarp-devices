# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusControlboard
                    CATEGORY device
                    TYPE roboticslab::CanBusControlboard
                    INCLUDE CanBusControlboard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_CanBusControlboard)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(CanBusControlboard CanBusControlboard.cpp
                                       CanBusControlboard.hpp
                                       DeviceDriverImpl.cpp
                                       IControlLimitsImpl.cpp
                                       IControlModeImpl.cpp
                                       ICurrentControlImpl.cpp
                                       IEncodersImpl.cpp
                                       IEncodersTimedImpl.cpp
                                       IInteractionModeImpl.cpp
                                       IPositionControlImpl.cpp
                                       IPositionDirectImpl.cpp
                                       ITorqueControlImpl.cpp
                                       IVelocityControlImpl.cpp
                                       ThreadImpl.cpp)

    target_link_libraries(CanBusControlboard YARP::YARP_OS
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             YarpDevicesInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS CanBusControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS CanBusControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES CanBusControlboard.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
