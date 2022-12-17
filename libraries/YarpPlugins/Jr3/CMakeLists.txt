find_path(JR3PCI_INCLUDE_DIR jr3pci-ioctl.h)
mark_as_advanced(JR3PCI_INCLUDE_DIR)

if((NOT UNIX OR NOT JR3PCI_INCLUDE_DIR) AND (NOT DEFINED ENABLE_Jr3 OR ENABLE_Jr3))
    message(WARNING "jr3pci-ioctl.h header not found, disabling Jr3")
endif()

yarp_prepare_plugin(Jr3
                    CATEGORY device
                    TYPE roboticslab::Jr3
                    INCLUDE Jr3.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;JR3PCI_INCLUDE_DIR"
                    EXTRA_CONFIG WRAPPER=analogServer)

if(NOT SKIP_Jr3)

    yarp_add_plugin(Jr3 Jr3.hpp
                        DeviceDriverImpl.cpp
                        IAnalogSensorImpl.cpp
                        ISixAxisForceTorqueSensorsImpl.cpp
                        LogComponent.hpp
                        LogComponent.cpp)

    target_link_libraries(Jr3 YARP::YARP_os
                              YARP::YARP_dev)

    target_include_directories(Jr3 PRIVATE ${JR3PCI_INCLUDE_DIR})

    yarp_install(TARGETS Jr3
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_Jr3 OFF CACHE BOOL "Enable/disable Jr3 device" FORCE)

endif()
