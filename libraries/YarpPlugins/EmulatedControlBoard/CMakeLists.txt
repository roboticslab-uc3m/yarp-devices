yarp_prepare_plugin(EmulatedControlBoard
                    CATEGORY device
                    TYPE roboticslab::EmulatedControlBoard
                    INCLUDE EmulatedControlBoard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if(NOT SKIP_EmulatedControlBoard)

    yarp_add_plugin(EmulatedControlBoard EmulatedControlBoard.hpp
                                         DeviceDriverImpl.cpp
                                         IControlLimitsImpl.cpp
                                         IControlModeImpl.cpp
                                         IEncodersImpl.cpp
                                         IPositionControlImpl.cpp
                                         IPositionDirectImpl.cpp
                                         IVelocityControlImpl.cpp
                                         PeriodicThreadImpl.cpp
                                         SharedArea.cpp
                                         LogComponent.hpp
                                         LogComponent.cpp)

    target_link_libraries(EmulatedControlBoard YARP::YARP_os
                                               YARP::YARP_dev)

    yarp_install(TARGETS EmulatedControlBoard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
