yarp_prepare_plugin(CanBusHico
                    CATEGORY device
                    TYPE roboticslab::CanBusHico
                    INCLUDE CanBusHico.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_CanBusHico)

    yarp_add_plugin(CanBusHico CanBusHico.cpp
                               CanBusHico.hpp
                               hico_api.h
                               DeviceDriverImpl.cpp
                               ICanBusImpl.cpp
                               ICanBusErrorsImpl.cpp
                               HicoCanMessage.cpp
                               HicoCanMessage.hpp
                               FilterManager.cpp
                               LogComponent.hpp
                               LogComponent.cpp)

    target_link_libraries(CanBusHico YARP::YARP_os
                                     YARP::YARP_dev)

    yarp_install(TARGETS CanBusHico
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
