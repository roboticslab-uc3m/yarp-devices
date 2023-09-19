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
                                 motor/IAmplifierControlImpl.cpp
                                 motor/IAxisInfoImpl.cpp
                                 motor/IControlCalibrationImpl.cpp
                                 motor/IControlLimitsImpl.cpp
                                 motor/IControlModeImpl.cpp
                                 motor/ICurrentControlImpl.cpp
                                 motor/IEncodersImpl.cpp
                                 motor/IImpedanceControlImpl.cpp
                                 motor/IInteractionModeImpl.cpp
                                 motor/IJointFaultImpl.cpp
                                 motor/IMotorImpl.cpp
                                 motor/IMotorEncodersImpl.cpp
                                 motor/IPidControlImpl.cpp
                                 motor/IPositionControlImpl.cpp
                                 motor/IPositionDirectImpl.cpp
                                 motor/IPWMControlImpl.cpp
                                 motor/IRemoteVariablesImpl.cpp
                                 motor/ITorqueControlImpl.cpp
                                 motor/IVelocityControlImpl.cpp
                                 # multiple analog sensors interfaces
                                 sensor/IThreeAxisGyroscopesImpl.cpp
                                 sensor/IThreeAxisLinearAccelerometersImpl.cpp
                                 sensor/IThreeAxisMagnetometersImpl.cpp
                                 sensor/IOrientationSensorsImpl.cpp
                                 sensor/ITemperatureSensorsImpl.cpp
                                 sensor/ISixAxisForceTorqueSensorsImpl.cpp
                                 sensor/IContactLoadCellArraysImpl.cpp
                                 sensor/IEncoderArraysImpl.cpp
                                 sensor/ISkinPatchesImpl.cpp
                                 sensor/IPositionSensorsImpl.cpp
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

    target_include_directories(CanBusBroker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

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
