if(NOT AMOR_API_FOUND AND (NOT DEFINED ENABLE_AmorControlBoard OR ENABLE_AmorControlBoard))
    message(WARNING "AMOR_API package not found, disabling AmorControlBoard")
endif()

yarp_prepare_plugin(AmorControlBoard
                    CATEGORY device
                    TYPE roboticslab::AmorControlBoard
                    INCLUDE AmorControlBoard.hpp
                    DEFAULT ON
                    DEPENDS AMOR_API_FOUND
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if(NOT SKIP_AmorControlBoard)

    yarp_add_plugin(AmorControlBoard AmorControlBoard.cpp
                                     AmorControlBoard.hpp
                                     DeviceDriverImpl.cpp
                                     IAxisInfoImpl.cpp
                                     IControlLimitsImpl.cpp
                                     IControlModeImpl.cpp
                                     ICurrentControlImpl.cpp
                                     IEncodersImpl.cpp
                                     IPositionControlImpl.cpp
                                     IVelocityControlImpl.cpp
                                     LogComponent.hpp
                                     LogComponent.cpp)

    target_link_libraries(AmorControlBoard YARP::YARP_os
                                           YARP::YARP_dev
                                           AMOR::amor_api)

    yarp_install(TARGETS AmorControlBoard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AmorControlBoard OFF CACHE BOOL "Enable/disable AmorControlBoard device" FORCE)

endif()
