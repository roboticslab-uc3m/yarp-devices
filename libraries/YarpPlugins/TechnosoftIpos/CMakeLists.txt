# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TechnosoftIpos
                    CATEGORY device
                    TYPE roboticslab::TechnosoftIpos
                    INCLUDE TechnosoftIpos.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_CanBusSharerLib)

if(NOT SKIP_TechnosoftIpos)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(TechnosoftIpos TechnosoftIpos.hpp
                                   TechnosoftIpos.cpp
                                   TechnosoftIposEmcy.cpp
                                   DeviceDriverImpl.cpp
                                   IAxisInfoRawImpl.cpp
                                   ICanBusSharerImpl.cpp
                                   IControlLimitsRawImpl.cpp
                                   IControlModeRawImpl.cpp
                                   ICurrentControlRawImpl.cpp
                                   IEncodersRawImpl.cpp
                                   IMotorRawImpl.cpp
                                   IMotorEncodersRawImpl.cpp
                                   IPositionControlRawImpl.cpp
                                   IPositionDirectRawImpl.cpp
                                   IRemoteVariablesRawImpl.cpp
                                   ITorqueControlRawImpl.cpp
                                   IVelocityControlRawImpl.cpp
                                   InterpolatedPositionBuffer.hpp
                                   InterpolatedPositionBuffer.cpp
                                   StateVariables.hpp
                                   StateVariables.cpp)

    target_link_libraries(TechnosoftIpos YARP::YARP_os
                                         YARP::YARP_dev
                                         ROBOTICSLAB::CanBusSharerLib
                                         ROBOTICSLAB::StateObserverLib
                                         ROBOTICSLAB::CanOpenNodeLib)

    target_compile_features(TechnosoftIpos PUBLIC cxx_std_14)

    yarp_install(TARGETS TechnosoftIpos
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
