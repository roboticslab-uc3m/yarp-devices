if(NOT Leap_FOUND AND (NOT DEFINED ENABLE_LeapMotionSensor OR ENABLE_LeapMotionSensor))
    message(WARNING "Leap package not found, disabling LeapMotionSensor")
endif()

yarp_prepare_plugin(LeapMotionSensor
                    CATEGORY device
                    TYPE LeapMotionSensor
                    INCLUDE LeapMotionSensor.hpp
                    DEFAULT ON
                    DEPENDS Leap_FOUND
                    EXTRA_CONFIG WRAPPER=analogServer)

if(NOT SKIP_LeapMotionSensor)

    yarp_add_plugin(LeapMotionSensor)

    target_sources(LeapMotionSensor PRIVATE LeapMotionSensor.hpp
                                            DeviceDriverImpl.cpp
                                            IAnalogSensorImpl.cpp
                                            LogComponent.hpp
                                            LogComponent.cpp)

    target_link_libraries(LeapMotionSensor YARP::YARP_os
                                           YARP::YARP_dev
                                           YARP::YARP_sig
                                           ${Leap_LIBRARY})

    target_include_directories(LeapMotionSensor PRIVATE ${Leap_INCLUDE_DIR})

    yarp_install(TARGETS LeapMotionSensor
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_LeapMotionSensor OFF CACHE BOOL "Enable/disable LeapMotionSensor device" FORCE)

endif()
