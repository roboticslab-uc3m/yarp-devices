# Copyright: (C) 2019 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Jennifer Joana Gago Muñoz
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraControlboardUSB
                    CATEGORY device
                    TYPE roboticslab::DextraControlboardUSB
                    INCLUDE DextraControlboardUSB.hpp
                    DEFAULT ON
                    DEPENDS UNIX
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_DextraControlboardUSB)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(DextraControlboardUSB DextraControlboardUSB.cpp
                                          DextraControlboardUSB.hpp
                                          DeviceDriverImpl.cpp
                                          IControlLimitsImpl.cpp
                                          IControlModeImpl.cpp
                                          IEncodersImpl.cpp
                                          IEncodersTimedImpl.cpp
                                          IInteractionModeImpl.cpp
                                          IPositionControlImpl.cpp
                                          IPositionDirectImpl.cpp
                                          ITorqueControlImpl.cpp
                                          IVelocityControlImpl.cpp
                                          Synapse.hpp
                                          Synapse.cpp)

    target_link_libraries(DextraControlboardUSB YARP::YARP_OS
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug
                                                YarpDevicesInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS DextraControlboardUSB
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS DextraControlboardUSB
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES DextraControlboardUSB.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
