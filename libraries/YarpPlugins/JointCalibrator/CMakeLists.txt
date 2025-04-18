yarp_prepare_plugin(JointCalibrator
                    CATEGORY device
                    TYPE JointCalibrator
                    INCLUDE JointCalibrator.hpp
                    DEFAULT ON
                    GENERATE_PARSER)

if(NOT SKIP_JointCalibrator)

    yarp_add_plugin(JointCalibrator)

    target_sources(JointCalibrator PRIVATE JointCalibrator.hpp
                                           DeviceDriverImpl.cpp
                                           IRemoteCalibratorImpl.cpp
                                           WrapperSingleImpl.cpp
                                           JointCalibrator_ParamsParser.h
                                           JointCalibrator_ParamsParser.cpp
                                           LogComponent.hpp
                                           LogComponent.cpp)

    target_link_libraries(JointCalibrator YARP::YARP_os
                                          YARP::YARP_dev)

    yarp_install(TARGETS JointCalibrator
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
