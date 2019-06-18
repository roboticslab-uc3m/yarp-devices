# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT Leap_FOUND AND (NOT DEFINED ENABLE_LeapMotionSensor OR ENABLE_LeapMotionSensor))
    message(WARNING "Leap package not found, disabling LeapMotionSensor")
endif()

yarp_prepare_plugin(LeapMotionSensor
                    CATEGORY device
                    TYPE roboticslab::LeapMotionSensor
                    INCLUDE LeapMotionSensor.hpp
                    DEFAULT ON
                    DEPENDS Leap_FOUND
                    EXTRA_CONFIG WRAPPER=analogServer)

if(NOT SKIP_LeapMotionSensor)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(LeapMotionSensor LeapMotionSensor.hpp
                                     DeviceDriverImpl.cpp
                                     IAnalogSensorImpl.cpp)

    target_link_libraries(LeapMotionSensor YARP::YARP_OS
                                           YARP::YARP_dev
                                           YARP::YARP_sig
                                           ${Leap_LIBRARY}
                                           ROBOTICSLAB::ColorDebug)

    target_include_directories(LeapMotionSensor PRIVATE ${Leap_INCLUDE_DIR})

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS LeapMotionSensor
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS LeapMotionSensor
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES LeapMotionSensor.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

else()

    set(ENABLE_LeapMotionSensor OFF CACHE BOOL "Enable/disable LeapMotionSensor device" FORCE)

endif()
