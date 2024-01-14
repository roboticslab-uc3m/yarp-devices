yarp_prepare_plugin(TechnosoftIpos
                    CATEGORY device
                    TYPE roboticslab::TechnosoftIpos
                    INCLUDE TechnosoftIpos.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_CanOpenNodeLib;ENABLE_StateObserverLib")

if(NOT SKIP_TechnosoftIpos)

    yarp_add_plugin(TechnosoftIpos)

    target_sources(TechnosoftIpos PRIVATE TechnosoftIpos.hpp
                                          TechnosoftIpos.cpp
                                          TechnosoftIposBase.hpp
                                          TechnosoftIposBase.cpp
                                          TechnosoftIposEmcy.hpp
                                          TechnosoftIposEmcy.cpp
                                          CommandBuffer.hpp
                                          CommandBuffer.cpp
                                          EncoderRead.hpp
                                          EncoderRead.cpp
                                          DeviceDriverImpl.cpp
                                          IWrapperImpl.cpp
                                          IAxisInfoRawImpl.cpp
                                          ICanBusSharerImpl.cpp
                                          IControlLimitsRawImpl.cpp
                                          ICurrentControlRawImpl.cpp
                                          IEncodersRawImpl.cpp
                                          IJointFaultRawImpl.cpp
                                          IMotorRawImpl.cpp
                                          IMotorEncodersRawImpl.cpp
                                          ITorqueControlRawImpl.cpp
                                          LogComponent.hpp
                                          LogComponent.cpp)

    target_sources(TechnosoftIpos PRIVATE embedded-pid/TechnosoftIposEmbedded.hpp
                                          embedded-pid/TechnosoftIposEmbedded.cpp
                                          embedded-pid/DeviceDriverImpl.cpp
                                          embedded-pid/InterpolatedPositionBuffer.hpp
                                          embedded-pid/InterpolatedPositionBuffer.cpp
                                          embedded-pid/ICanBusSharerImpl.cpp
                                          embedded-pid/IControlModeRawImpl.cpp
                                          embedded-pid/IPositionControlRawImpl.cpp
                                          embedded-pid/IPositionDirectRawImpl.cpp
                                          embedded-pid/IRemoteVariablesRawImpl.cpp
                                          embedded-pid/IVelocityControlRawImpl.cpp)

    if(ENABLE_TrajectoryLib)
        target_sources(TechnosoftIpos PRIVATE external-pid/TechnosoftIposExternal.hpp
                                              external-pid/TechnosoftIposExternal.cpp
                                              external-pid/DeviceDriverImpl.cpp
                                              external-pid/ICanBusSharerImpl.cpp
                                              external-pid/IControlModeRawImpl.cpp
                                              external-pid/IImpedanceControlRawImpl.cpp
                                              external-pid/IInteractionModeRawImpl.cpp
                                              external-pid/IPidControlRawImpl.cpp
                                              external-pid/IPositionControlRawImpl.cpp
                                              external-pid/IPositionDirectRawImpl.cpp
                                              external-pid/IRemoteVariablesRawImpl.cpp
                                              external-pid/IVelocityControlRawImpl.cpp)

        target_link_libraries(TechnosoftIpos ROBOTICSLAB::TrajectoryLib)
        target_compile_definitions(TechnosoftIpos PRIVATE HAVE_EXTERNAL_PID_IMPL)
    endif()

    target_include_directories(TechnosoftIpos PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    target_link_libraries(TechnosoftIpos YARP::YARP_os
                                         YARP::YARP_dev
                                         ROBOTICSLAB::CanBusSharerLib
                                         ROBOTICSLAB::CanOpenNodeLib
                                         ROBOTICSLAB::StateObserverLib)

    yarp_install(TARGETS TechnosoftIpos
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
