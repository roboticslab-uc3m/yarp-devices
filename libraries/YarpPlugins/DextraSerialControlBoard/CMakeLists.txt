yarp_prepare_plugin(DextraSerialControlBoard
                    CATEGORY device
                    TYPE roboticslab::DextraSerialControlBoard
                    INCLUDE DextraSerialControlBoard.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_DextraRawControlBoardLib
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if(NOT SKIP_DextraSerialControlBoard)

    yarp_add_plugin(DextraSerialControlBoard DextraSerialControlBoard.hpp
                                             DeviceDriverImpl.cpp
                                             SerialSynapse.cpp)

    target_link_libraries(DextraSerialControlBoard ROBOTICSLAB::DextraRawControlBoardLib
                                                   YARP::YARP_os
                                                   YARP::YARP_dev)

    yarp_install(TARGETS DextraSerialControlBoard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
