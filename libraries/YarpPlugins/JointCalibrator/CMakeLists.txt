# Copyright: (C) 2019 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(JointCalibrator
                    CATEGORY device
                    TYPE roboticslab::JointCalibrator
                    INCLUDE JointCalibrator.hpp
                    DEFAULT ON)

if(NOT SKIP_JointCalibrator)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(JointCalibrator JointCalibrator.hpp
                                    DeviceDriverImpl.cpp
                                    IRemoteCalibratorImpl.cpp
                                    IWrapperImpl.cpp)

    target_link_libraries(JointCalibrator YARP::YARP_OS
                                          YARP::YARP_dev
                                          ROBOTICSLAB::ColorDebug)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS JointCalibrator
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS JointCalibrator
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES JointCalibrator.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
