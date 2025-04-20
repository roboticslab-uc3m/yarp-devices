yarp_prepare_plugin(CanBusSocket
                    CATEGORY device
                    TYPE CanBusSocket
                    INCLUDE CanBusSocket.hpp
                    DEFAULT ON
                    DEPENDS UNIX
                    GENERATE_PARSER)

if(NOT SKIP_CanBusSocket)

    yarp_add_plugin(CanBusSocket)

    target_sources(CanBusSocket PRIVATE CanBusSocket.cpp
                                        CanBusSocket.hpp
                                        DeviceDriverImpl.cpp
                                        ICanBusImpl.cpp
                                        SocketCanMessage.cpp
                                        SocketCanMessage.hpp
                                        CanBusSocket_ParamsParser.h
                                        CanBusSocket_ParamsParser.cpp
                                        LogComponent.hpp
                                        LogComponent.cpp)

    target_link_libraries(CanBusSocket YARP::YARP_os
                                       YARP::YARP_dev)

    yarp_install(TARGETS CanBusSocket
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
