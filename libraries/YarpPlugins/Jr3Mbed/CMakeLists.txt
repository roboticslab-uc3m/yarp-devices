yarp_prepare_plugin(Jr3Mbed
                    CATEGORY device
                    TYPE roboticslab::Jr3Mbed
                    INCLUDE Jr3Mbed.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_StateObserverLib")

if(NOT SKIP_Jr3Mbed)

    yarp_add_plugin(Jr3Mbed Jr3Mbed.hpp
                            Jr3Mbed.cpp
                            DeviceDriverImpl.cpp
                            ICanBusSharerImpl.cpp
                            ISixAxisForceTorqueSensorsImpl.cpp
                            LogComponent.hpp
                            LogComponent.cpp)

    target_link_libraries(Jr3Mbed YARP::YARP_os
                                  YARP::YARP_dev
                                  ROBOTICSLAB::CanBusSharerLib
                                  ROBOTICSLAB::StateObserverLib)

    yarp_install(TARGETS Jr3Mbed
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_Jr3Mbed OFF CACHE BOOL "Enable/disable Jr3Mbed device" FORCE)

endif()
