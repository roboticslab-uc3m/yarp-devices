yarp_prepare_plugin(CuiAbsolute
                    CATEGORY device
                    TYPE CuiAbsolute
                    INCLUDE CuiAbsolute.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_StateObserverLib"
                    GENERATE_PARSER)

if(NOT SKIP_CuiAbsolute)

    yarp_add_plugin(CuiAbsolute)

    target_sources(CuiAbsolute PRIVATE CuiAbsolute.hpp
                                       CuiAbsolute.cpp
                                       DeviceDriverImpl.cpp
                                       ICanBusSharerImpl.cpp
                                       IEncodersRawImpl.cpp
                                       CuiAbsolute_ParamsParser.h
                                       CuiAbsolute_ParamsParser.cpp
                                       LogComponent.hpp
                                       LogComponent.cpp)

    target_link_libraries(CuiAbsolute YARP::YARP_os
                                      YARP::YARP_dev
                                      ROBOTICSLAB::CanBusSharerLib
                                      ROBOTICSLAB::StateObserverLib)

    yarp_install(TARGETS CuiAbsolute
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
