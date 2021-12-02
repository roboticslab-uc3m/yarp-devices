# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusControlboard
                    CATEGORY device
                    TYPE roboticslab::CanBusControlboard
                    INCLUDE CanBusControlboard.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_YarpDeviceMapperLib"
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_CanBusControlboard)

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
                                       CanBusBroker.hpp
                                       CanBusBroker.cpp
                                       CanRxTxThreads.hpp
                                       CanRxTxThreads.cpp
                                       SdoReplier.hpp
                                       SdoReplier.cpp
                                       BusLoadMonitor.hpp
                                       BusLoadMonitor.cpp
                                       YarpCanSenderDelegate.hpp
                                       YarpCanSenderDelegate.cpp
                                       SyncPeriodicThread.hpp
                                       SyncPeriodicThread.cpp
                                       LogComponent.hpp
                                       LogComponent.cpp)

    if(YARP_VERSION_SHORT VERSION_GREATER_EQUAL 3.5.100)
        target_sources(CanBusControlboard PRIVATE IJointFaultImpl.cpp)
    endif()

    target_link_libraries(CanBusControlboard YARP::YARP_os
                                             YARP::YARP_dev
                                             ROBOTICSLAB::CanBusSharerLib
                                             ROBOTICSLAB::CanOpenNodeLib
                                             ROBOTICSLAB::StateObserverLib
                                             ROBOTICSLAB::YarpDeviceMapperLib)

    target_compile_features(CanBusControlboard PRIVATE cxx_std_14)

    yarp_install(TARGETS CanBusControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
