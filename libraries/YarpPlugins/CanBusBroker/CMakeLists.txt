yarp_prepare_plugin(CanBusBroker
                    CATEGORY device
                    TYPE roboticslab::CanBusBroker
                    INCLUDE CanBusBroker.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_CanOpenNodeLib;ENABLE_StateObserverLib;ENABLE_YarpDeviceMapperLib"
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if(NOT SKIP_CanBusBroker)

    yarp_add_plugin(CanBusBroker CanBusBroker.hpp
                                 DeviceDriverImpl.cpp
                                 # control board interfaces
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
                                 # multiple analog sensors interfaces
                                 IThreeAxisGyroscopesImpl.cpp
                                 IThreeAxisLinearAccelerometersImpl.cpp
                                 IThreeAxisMagnetometersImpl.cpp
                                 IOrientationSensorsImpl.cpp
                                 ITemperatureSensorsImpl.cpp
                                 ISixAxisForceTorqueSensorsImpl.cpp
                                 IContactLoadCellArraysImpl.cpp
                                 IEncoderArraysImpl.cpp
                                 ISkinPatchesImpl.cpp
                                 IPositionSensorsImpl.cpp
                                 # broker, NWS and misc stuff
                                 CanRxTxThreads.hpp
                                 CanRxTxThreads.cpp
                                 SdoReplier.hpp
                                 SdoReplier.cpp
                                 SingleBusBroker.hpp
                                 SingleBusBroker.cpp
                                 BusLoadMonitor.hpp
                                 BusLoadMonitor.cpp
                                 YarpCanSenderDelegate.hpp
                                 YarpCanSenderDelegate.cpp
                                 SyncPeriodicThread.hpp
                                 SyncPeriodicThread.cpp
                                 LogComponent.hpp
                                 LogComponent.cpp)

    target_link_libraries(CanBusBroker YARP::YARP_os
                                       YARP::YARP_dev
                                       ROBOTICSLAB::CanBusSharerLib
                                       ROBOTICSLAB::CanOpenNodeLib
                                       ROBOTICSLAB::StateObserverLib
                                       ROBOTICSLAB::YarpDeviceMapperLib)

    yarp_install(TARGETS CanBusBroker
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
