yarp_prepare_plugin(DextraSerialControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraSerialControlboard
                    INCLUDE DextraSerialControlboard.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_DextraRawControlboardLib
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if(NOT SKIP_DextraSerialControlboard)

    yarp_add_plugin(DextraSerialControlboard DextraSerialControlboard.hpp
                                             DeviceDriverImpl.cpp
                                             SerialSynapse.cpp)

    target_link_libraries(DextraSerialControlboard ROBOTICSLAB::DextraRawControlboardLib
                                                   YARP::YARP_os
                                                   YARP::YARP_dev)

    yarp_install(TARGETS DextraSerialControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
