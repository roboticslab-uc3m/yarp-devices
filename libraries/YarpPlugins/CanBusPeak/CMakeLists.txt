if(NOT PCan_FOUND AND (NOT DEFINED ENABLE_CanBusPeak OR ENABLE_CanBusPeak))
    message(WARNING "PCan package not found, disabling CanBusPeak")
endif()

yarp_prepare_plugin(CanBusPeak
                    CATEGORY device
                    TYPE CanBusPeak
                    INCLUDE CanBusPeak.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;PCan_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_CanBusPeak)

    yarp_add_plugin(CanBusPeak)

    target_sources(CanBusPeak PRIVATE CanBusPeak.cpp
                                      CanBusPeak.hpp
                                      DeviceDriverImpl.cpp
                                      ICanBusImpl.cpp
                                      ICanBusErrorsImpl.cpp
                                      PeakCanMessage.cpp
                                      PeakCanMessage.hpp
                                      CanBusPeak_ParamsParser.h
                                      CanBusPeak_ParamsParser.cpp
                                      LogComponent.hpp
                                      LogComponent.cpp)

    target_link_libraries(CanBusPeak YARP::YARP_os
                                     YARP::YARP_dev
                                     PCan::PCanFD)

    yarp_install(TARGETS CanBusPeak
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_CanBusPeak OFF CACHE BOOL "Enable/disable CanBusPeak device" FORCE)

endif()
