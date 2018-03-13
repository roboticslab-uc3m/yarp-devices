# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(AMOR_API QUIET)

if(NOT AMOR_API_FOUND AND (NOT DEFINED ENABLE_AmorControlboard OR ENABLE_AmorControlboard))
    message(WARNING "AMOR_API package not found, disabling AmorControlboard")
endif()

yarp_prepare_plugin(AmorControlboard
                    CATEGORY device
                    TYPE roboticslab::AmorControlboard
                    INCLUDE AmorControlboard.hpp
                    DEFAULT ON
                    DEPENDS AMOR_API_FOUND)

if(NOT SKIP_AmorControlboard)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this)

    yarp_add_plugin(AmorControlboard AmorControlboard.cpp
                                     AmorControlboard.hpp
                                     DeviceDriverImpl.cpp
                                     IControlLimitsImpl.cpp
                                     IControlLimits2Impl.cpp
                                     IControlModeImpl.cpp
                                     IControlMode2Impl.cpp
                                     IEncodersImpl.cpp
                                     IEncodersTimedImpl.cpp
                                     IPositionControlImpl.cpp
                                     IPositionControl2Impl.cpp
                                     IVelocityControlImpl.cpp
                                     IVelocityControl2Impl.cpp
                                     IAxisInfoImpl.cpp
                                     ITorqueControlImpl.cpp
                                     IInteractionModeImpl.cpp)

    target_link_libraries(AmorControlboard YARP::YARP_OS
                                           YARP::YARP_dev
                                           AMOR::amor_api)

    yarp_install(TARGETS AmorControlboard
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES AmorControlboard.ini
                 COMPONENT runtime
                 DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AmorControlboard OFF CACHE BOOL "Enable/disable AmorControlboard device" FORCE)

endif()
