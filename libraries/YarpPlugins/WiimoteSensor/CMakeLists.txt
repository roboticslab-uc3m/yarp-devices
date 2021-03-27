# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT XWiimote_FOUND AND (NOT DEFINED ENABLE_WiimoteSensor OR ENABLE_WiimoteSensor))
    message(WARNING "XWiimote package not found, disabling WiimoteSensor")
endif()

yarp_prepare_plugin(WiimoteSensor
                    CATEGORY device
                    TYPE roboticslab::WiimoteSensor
                    INCLUDE WiimoteSensor.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;XWiimote_FOUND"
                    EXTRA_CONFIG WRAPPER=analogServer)

if(NOT SKIP_WiimoteSensor)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(WiimoteSensor WiimoteSensor.hpp
                                  WiimoteSensor.cpp
                                  DeviceDriverImpl.cpp
                                  IAnalogSensorImpl.cpp
                                  WiimoteDispatcherThread.cpp)

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
