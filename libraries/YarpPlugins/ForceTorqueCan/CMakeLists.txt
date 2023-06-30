yarp_prepare_plugin(ForceTorqueCan
                    CATEGORY device
                    TYPE roboticslab::ForceTorqueCan
                    INCLUDE ForceTorqueCan.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=multipleanalogsensorsserver)

if(NOT SKIP_ForceTorqueCan)

    yarp_add_plugin(ForceTorqueCan ForceTorqueCan.hpp
                                   DeviceDriverImpl.cpp
                                   ISixAxisForceTorqueSensorsImpl.cpp
                                   ThreadImpl.cpp
                                   LogComponent.hpp
                                   LogComponent.cpp)

    target_link_libraries(ForceTorqueCan YARP::YARP_os
                                         YARP::YARP_dev)

    yarp_install(TARGETS ForceTorqueCan
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_ForceTorqueCan OFF CACHE BOOL "Enable/disable ForceTorqueCan device" FORCE)

endif()
