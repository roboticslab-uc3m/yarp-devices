yarp_prepare_plugin(CanBusControlboard
                    CATEGORY device
                    TYPE roboticslab::CanBusControlboard
                    INCLUDE CanBusControlboard.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_YarpDeviceMapperLib"
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

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
                                       IJointFaultImpl.cpp
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

    target_link_libraries(CanBusControlboard YARP::YARP_os
                                             YARP::YARP_dev
                                             ROBOTICSLAB::CanBusSharerLib
                                             ROBOTICSLAB::CanOpenNodeLib
                                             ROBOTICSLAB::StateObserverLib
                                             ROBOTICSLAB::YarpDeviceMapperLib)

    yarp_install(TARGETS CanBusControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
