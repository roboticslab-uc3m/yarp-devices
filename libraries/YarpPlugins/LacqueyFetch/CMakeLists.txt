yarp_prepare_plugin(LacqueyFetch
                    CATEGORY device
                    TYPE roboticslab::LacqueyFetch
                    INCLUDE LacqueyFetch.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_CanBusSharerLib)

if(NOT SKIP_LacqueyFetch)

    yarp_add_plugin(LacqueyFetch LacqueyFetch.hpp
                                 DeviceDriverImpl.cpp
                                 IAxisInfoRawImpl.cpp
                                 ICanBusSharerImpl.cpp
                                 IControlModeRawImpl.cpp
                                 IPWMControlRawImpl.cpp
                                 ISixAxisForceTorqueSensorsImpl.cpp
                                 LogComponent.hpp
                                 LogComponent.cpp)

    target_link_libraries(LacqueyFetch YARP::YARP_os
                                       YARP::YARP_dev
                                       ROBOTICSLAB::CanBusSharerLib)

    yarp_install(TARGETS LacqueyFetch
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
