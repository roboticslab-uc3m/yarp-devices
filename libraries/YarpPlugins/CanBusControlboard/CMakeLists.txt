# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusControlboard
                    CATEGORY device
                    TYPE roboticslab::CanBusControlboard
                    INCLUDE CanBusControlboard.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_DeviceMapperLib"
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_CanBusControlboard)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(CanBusControlboard CanBusControlboard.hpp
                                       DeviceDriverImpl.cpp
                                       IAmplifierControlImpl.cpp
                                       IAxisInfoImpl.cpp
                                       IControlCalibrationImpl.cpp
                                       IControlLimitsImpl.cpp
                                       IControlModeImpl.cpp
                                       ICurrentControlImpl.cpp
                                       IEncodersImpl.cpp
                                       IImpedanceControlImpl.cpp
                                       IInteractionModeImpl.cpp
                                       IMotorImpl.cpp
                                       IMotorEncodersImpl.cpp
                                       IPidControlImpl.cpp
                                       IPositionControlImpl.cpp
                                       IPositionDirectImpl.cpp
                                       IPWMControlImpl.cpp
                                       IRemoteVariablesImpl.cpp
                                       ITorqueControlImpl.cpp
                                       IVelocityControlImpl.cpp
                                       PositionDirectThread.hpp
                                       PositionDirectThread.cpp
                                       CanRxTxThreads.hpp
                                       CanRxTxThreads.cpp
                                       YarpCanSenderDelegate.hpp
                                       YarpCanSenderDelegate.cpp)

    target_link_libraries(CanBusControlboard YARP::YARP_OS
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             CanBusSharerLib
                                             DeviceMapperLib
                                             YarpDevicesInterfaces)

    yarp_install(TARGETS CanBusControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
