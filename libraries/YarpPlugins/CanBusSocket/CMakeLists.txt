yarp_prepare_plugin(CanBusSocket
                    CATEGORY device
                    TYPE roboticslab::CanBusSocket
                    INCLUDE CanBusSocket.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_CanBusSocket)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs})

yarp_add_plugin(CanBusSocket CanBusSocket.cpp
                             CanBusSocket.hpp
                             DeviceDriverImpl.cpp
                             ICanBusImpl.cpp
                             SocketCanMessage.cpp
                             SocketCanMessage.hpp)

add_dependencies(CanBusSocket COLOR_DEBUG)

target_link_libraries(CanBusSocket YARP::YARP_OS
                                   YARP::YARP_dev)

yarp_install(TARGETS CanBusSocket
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CanBusSocket.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
