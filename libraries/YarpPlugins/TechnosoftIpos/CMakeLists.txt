# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TechnosoftIpos
                    CATEGORY device
                    TYPE roboticslab::TechnosoftIpos
                    INCLUDE TechnosoftIpos.hpp
                    DEFAULT ON)

if(NOT SKIP_TechnosoftIpos)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(TechnosoftIpos TechnosoftIpos.cpp
                                   TechnosoftIpos.hpp
                                   DeviceDriverImpl.cpp
                                   ICanBusSharerImpl.cpp
                                   IControlLimitsRawImpl.cpp
                                   IControlModeRawImpl.cpp
                                   ICurrentControlRawImpl.cpp
                                   IEncodersRawImpl.cpp
                                   IEncodersTimedRawImpl.cpp
                                   IInteractionModeRawImpl.cpp
                                   IPositionControlRawImpl.cpp
                                   IPositionDirectRawImpl.cpp
                                   IRemoteVariablesRawImpl.cpp
                                   ITorqueControlRawImpl.cpp
                                   IVelocityControlRawImpl.cpp
                                   LinearInterpolationBuffer.hpp
                                   LinearInterpolationBuffer.cpp)

    target_link_libraries(TechnosoftIpos YARP::YARP_OS
                                         YARP::YARP_dev
                                         ROBOTICSLAB::ColorDebug
                                         YarpDevicesInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS TechnosoftIpos
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS TechnosoftIpos
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES TechnosoftIpos.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
