yarp_prepare_plugin(TextilesHand
                    CATEGORY device
                    TYPE TextilesHand
                    INCLUDE TextilesHand.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp
                    GENERATE_PARSER)

if(NOT SKIP_TextilesHand)

    yarp_add_plugin(TextilesHand)

    target_sources(TextilesHand PRIVATE TextilesHand.hpp
                                        DeviceDriverImpl.cpp
                                        IControlModeImpl.cpp
                                        IPositionDirectImpl.cpp
                                        TextilesHand_ParamsParser.h
                                        TextilesHand_ParamsParser.cpp
                                        LogComponent.hpp
                                        LogComponent.cpp)

    target_link_libraries(TextilesHand YARP::YARP_os
                                       YARP::YARP_dev)

    yarp_install(TARGETS TextilesHand
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
