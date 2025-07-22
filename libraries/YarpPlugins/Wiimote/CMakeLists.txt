if(NOT XWiimote_FOUND AND (NOT DEFINED ENABLE_Wiimote OR ENABLE_Wiimote))
    message(WARNING "XWiimote package not found, disabling Wiimote")
endif()

yarp_prepare_plugin(Wiimote
                    CATEGORY device
                    TYPE Wiimote
                    INCLUDE Wiimote.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;XWiimote_FOUND"
                    EXTRA_CONFIG WRAPPER=JoypadControlServer
                    GENERATE_PARSER)

if(NOT SKIP_Wiimote)

    yarp_add_plugin(Wiimote)

    target_sources(Wiimote PRIVATE Wiimote.hpp
                                   Wiimote.cpp
                                   DeviceDriverImpl.cpp
                                   IJoypadControllerImpl.cpp
                                   WiimoteDispatcherThread.cpp
                                   Wiimote_ParamsParser.h
                                   Wiimote_ParamsParser.cpp
                                   LogComponent.hpp
                                   LogComponent.cpp)

    target_link_libraries(Wiimote YARP::YARP_os
                                  YARP::YARP_dev
                                  XWiimote::XWiimote)

    if(XWiimote_VERSION VERSION_LESS "2")
        target_compile_definitions(Wiimote PRIVATE _XWIIMOTE_LEGACY_INTERFACE)
    endif()

    yarp_install(TARGETS Wiimote
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_Wiimote OFF CACHE BOOL "Enable/disable Wiimote device" FORCE)

endif()
