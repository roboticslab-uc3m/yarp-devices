if(NOT Spnav_FOUND AND (NOT DEFINED ENABLE_SpaceNavigator OR ENABLE_SpaceNavigator))
    message(WARNING "Spnav package not found, disabling SpaceNavigator")
endif()

yarp_prepare_plugin(SpaceNavigator
                    CATEGORY device
                    TYPE SpaceNavigator
                    INCLUDE SpaceNavigator.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;Spnav_FOUND"
                    EXTRA_CONFIG WRAPPER=analogServer
                    GENERATE_PARSER)

if(NOT SKIP_SpaceNavigator)

    yarp_add_plugin(SpaceNavigator)

    target_sources(SpaceNavigator PRIVATE SpaceNavigator.hpp
                                          DeviceDriverImpl.cpp
                                          IAnalogSensorImpl.cpp
                                          SpaceNavigator_ParamsParser.h
                                          SpaceNavigator_ParamsParser.cpp)

    target_link_libraries(SpaceNavigator YARP::YARP_os
                                         YARP::YARP_dev
                                         ${SPNAV_LIBRARIES})

    target_include_directories(SpaceNavigator PRIVATE ${SPNAV_INCLUDE_DIRS})

    yarp_install(TARGETS SpaceNavigator
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_SpaceNavigator OFF CACHE BOOL "Enable/disable SpaceNavigator device" FORCE)

endif()
