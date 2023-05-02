yarp_prepare_plugin(FakeJoint
                    CATEGORY device
                    TYPE roboticslab::FakeJoint
                    INCLUDE FakeJoint.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_CanBusSharerLib)

if(NOT SKIP_FakeJoint)

    yarp_add_plugin(FakeJoint FakeJoint.hpp
                              FakeJoint.cpp)

    target_link_libraries(FakeJoint YARP::YARP_dev
                                    ROBOTICSLAB::CanBusSharerLib)

    yarp_install(TARGETS FakeJoint
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
