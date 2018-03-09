# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(XWiimote 0.3 QUIET)

if(NOT XWiimote_FOUND AND (NOT DEFINED ENABLE_WiimoteSensor OR ENABLE_WiimoteSensor))
    message(WARNING "XWiimote package not found, disabling WiimoteSensor")
endif()

yarp_prepare_plugin(WiimoteSensor
                    CATEGORY device
                    TYPE roboticslab::WiimoteSensor
                    INCLUDE WiimoteSensor.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;NOT CMAKE_VERSION VERSION_LESS 2.8.11;XWiimote_FOUND")

if(NOT SKIP_WiimoteSensor)

    if(XWiimote_VERSION VERSION_LESS "2")
        add_definitions(-DXWIIMOTE_LEGACY_INTERFACE)
    endif()

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

    yarp_add_plugin(WiimoteSensor
              WiimoteSensor.hpp
              WiimoteSensor.cpp
              DeviceDriverImpl.cpp
              IAnalogSensorImpl.cpp
              WiimoteDispatcherThread.cpp)

    add_dependencies(WiimoteSensor COLOR_DEBUG)

    target_link_libraries(WiimoteSensor YARP::YARP_OS
                                        YARP::YARP_dev
                                        XWiimote::XWiimote
                                        YarpDevicesInterfaces)

    yarp_install(TARGETS WiimoteSensor
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES WiimoteSensor.ini
                 COMPONENT runtime
                 DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_WiimoteSensor OFF CACHE BOOL "Enable/disable WiimoteSensor device" FORCE)

endif()
