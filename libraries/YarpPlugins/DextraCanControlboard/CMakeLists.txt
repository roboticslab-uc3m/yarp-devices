yarp_prepare_plugin(DextraCanControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraCanControlboard
                    INCLUDE DextraCanControlboard.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_DextraRawControlboardLib")

if(NOT SKIP_DextraCanControlboard)

    yarp_add_plugin(DextraCanControlboard DextraCanControlboard.hpp
                                          DeviceDriverImpl.cpp
                                          ICanBusSharerImpl.cpp
                                          CanSynapse.cpp)

    target_link_libraries(DextraCanControlboard ROBOTICSLAB::DextraRawControlboardLib
                                                YARP::YARP_os
                                                YARP::YARP_dev
                                                ROBOTICSLAB::CanBusSharerLib)

    yarp_install(TARGETS DextraCanControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
