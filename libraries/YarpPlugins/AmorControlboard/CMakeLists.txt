# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT AMOR_API_FOUND AND (NOT DEFINED ENABLE_AmorControlboard OR ENABLE_AmorControlboard))
    message(WARNING "AMOR_API package not found, disabling AmorControlboard")
endif()

yarp_prepare_plugin(AmorControlboard
                    CATEGORY device
                    TYPE roboticslab::AmorControlboard
                    INCLUDE AmorControlboard.hpp
                    DEFAULT ON
                    DEPENDS AMOR_API_FOUND
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_AmorControlboard)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(AmorControlboard AmorControlboard.cpp
                                     AmorControlboard.hpp
                                     DeviceDriverImpl.cpp
                                     IAxisInfoImpl.cpp
                                     IControlLimitsImpl.cpp
                                     IControlModeImpl.cpp
                                     ICurrentControlImpl.cpp
                                     IEncodersImpl.cpp
                                     IEncodersTimedImpl.cpp
                                     IInteractionModeImpl.cpp
                                     IPositionControlImpl.cpp
                                     IRemoteVariablesImpl.cpp
                                     ITorqueControlImpl.cpp
                                     IVelocityControlImpl.cpp)

    target_link_libraries(AmorControlboard YARP::YARP_OS
                                           YARP::YARP_dev
                                           AMOR::amor_api
                                           ROBOTICSLAB::ColorDebug)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS AmorControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS AmorControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES AmorControlboard.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

else()

    set(ENABLE_AmorControlboard OFF CACHE BOOL "Enable/disable AmorControlboard device" FORCE)

endif()
