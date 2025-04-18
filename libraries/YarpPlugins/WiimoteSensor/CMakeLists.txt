if(NOT XWiimote_FOUND AND (NOT DEFINED ENABLE_WiimoteSensor OR ENABLE_WiimoteSensor))
    message(WARNING "XWiimote package not found, disabling WiimoteSensor")
endif()

yarp_prepare_plugin(WiimoteSensor
                    CATEGORY device
                    TYPE WiimoteSensor
                    INCLUDE WiimoteSensor.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;XWiimote_FOUND"
                    EXTRA_CONFIG WRAPPER=analogServer
                    GENERATE_PARSER)

if(NOT SKIP_WiimoteSensor)

    yarp_add_plugin(WiimoteSensor)

    target_sources(WiimoteSensor PRIVATE WiimoteSensor.hpp
                                         WiimoteSensor.cpp
                                         DeviceDriverImpl.cpp
                                         IAnalogSensorImpl.cpp
                                         WiimoteDispatcherThread.cpp
                                         WiimoteSensor_ParamsParser.h
                                         WiimoteSensor_ParamsParser.cpp
                                         LogComponent.hpp
                                         LogComponent.cpp)

    target_link_libraries(WiimoteSensor YARP::YARP_os
                                        YARP::YARP_dev
                                        XWiimote::XWiimote)

    if(XWiimote_VERSION VERSION_LESS "2")
        target_compile_definitions(WiimoteSensor PRIVATE XWIIMOTE_LEGACY_INTERFACE)
    endif()

    yarp_install(TARGETS WiimoteSensor
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_WiimoteSensor OFF CACHE BOOL "Enable/disable WiimoteSensor device" FORCE)

endif()
