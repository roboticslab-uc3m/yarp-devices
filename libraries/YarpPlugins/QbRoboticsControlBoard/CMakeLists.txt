yarp_prepare_plugin(QbRoboticsControlBoard
                    CATEGORY device
                    TYPE roboticslab::QbRoboticsControlBoard
                    INCLUDE QbRoboticsControlBoard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if(NOT SKIP_QbRoboticsControlBoard)

    yarp_add_plugin(QbRoboticsControlBoard QbRoboticsControlBoard.hpp
                                           DeviceDriverImpl.cpp
                                           IEncodersImpl.cpp
                                           IPositionControlImpl.cpp
                                           IVelocityControlImpl.cpp
                                           LogComponent.hpp
                                           LogComponent.cpp)

    target_link_libraries(QbRoboticsControlBoard YARP::YARP_os
                                                 YARP::YARP_dev)

    yarp_install(TARGETS QbRoboticsControlBoard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
