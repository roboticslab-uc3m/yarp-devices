find_path(JR3PCI_INCLUDE_DIR jr3pci-ioctl.h)
mark_as_advanced(JR3PCI_INCLUDE_DIR)

if((NOT UNIX OR NOT JR3PCI_INCLUDE_DIR) AND (NOT DEFINED ENABLE_Jr3Pci OR ENABLE_Jr3Pci))
    message(WARNING "jr3pci-ioctl.h header not found, disabling Jr3Pci")
endif()

yarp_prepare_plugin(Jr3Pci
                    CATEGORY device
                    TYPE Jr3Pci
                    INCLUDE Jr3Pci.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;JR3PCI_INCLUDE_DIR"
                    EXTRA_CONFIG WRAPPER=multipleanalogsensorsserver
                    GENERATE_PARSER)

if(NOT SKIP_Jr3Pci)

    yarp_add_plugin(Jr3Pci)

    target_sources(Jr3Pci PRIVATE Jr3Pci.hpp
                                  Jr3Pci.cpp
                                  DeviceDriverImpl.cpp
                                  ISixAxisForceTorqueSensorsImpl.cpp
                                  Jr3Pci_ParamsParser.h
                                  Jr3Pci_ParamsParser.cpp
                                  LogComponent.hpp
                                  LogComponent.cpp)

    target_link_libraries(Jr3Pci YARP::YARP_os
                                 YARP::YARP_dev)

    target_include_directories(Jr3Pci PRIVATE ${JR3PCI_INCLUDE_DIR})

    yarp_install(TARGETS Jr3Pci
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_Jr3Pci OFF CACHE BOOL "Enable/disable Jr3Pci device" FORCE)

endif()
