if(NOT PCan_FOUND AND (NOT DEFINED ENABLE_CanBusPeak OR ENABLE_CanBusPeak))
    message(WARNING "PCan package not found, disabling CanBusPeak")
endif()

yarp_prepare_plugin(CanBusPeak
                    CATEGORY device
                    TYPE roboticslab::CanBusPeak
                    INCLUDE CanBusPeak.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;PCan_FOUND")

if(NOT SKIP_CanBusPeak)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(CanBusPeak CanBusPeak.cpp
                               CanBusPeak.hpp
                               DeviceDriverImpl.cpp
                               ICanBusImpl.cpp
                               PeakCanMessage.cpp
                               PeakCanMessage.hpp)

    target_link_libraries(CanBusPeak YARP::YARP_OS
                                     YARP::YARP_dev
                                     PCan::PCanFD
                                     ROBOTICSLAB::ColorDebug
                                     YarpDevicesInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS CanBusPeak
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS CanBusPeak
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES CanBusPeak.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

else()

    set(ENABLE_CanBusPeak OFF CACHE BOOL "Enable/disable CanBusPeak device" FORCE)

endif()
