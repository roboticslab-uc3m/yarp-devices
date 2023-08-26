yarp_prepare_plugin(DextraCanControlBoard
                    CATEGORY device
                    TYPE roboticslab::DextraCanControlBoard
                    INCLUDE DextraCanControlBoard.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_DextraRawControlBoardLib")

if(NOT SKIP_DextraCanControlBoard)

    yarp_add_plugin(DextraCanControlBoard DextraCanControlBoard.hpp
                                          DeviceDriverImpl.cpp
                                          ICanBusSharerImpl.cpp
                                          CanSynapse.cpp)

    target_link_libraries(DextraCanControlBoard ROBOTICSLAB::DextraRawControlBoardLib
                                                YARP::YARP_os
                                                YARP::YARP_dev
                                                ROBOTICSLAB::CanBusSharerLib)

    yarp_install(TARGETS DextraCanControlBoard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
