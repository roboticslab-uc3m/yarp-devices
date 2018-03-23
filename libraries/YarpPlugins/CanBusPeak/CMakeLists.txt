yarp_prepare_plugin(CanBusPeak
                    CATEGORY device
                    TYPE roboticslab::CanBusPeak
                    INCLUDE CanBusPeak.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;NOT CMAKE_VERSION VERSION_LESS 2.8.11")

if(NOT SKIP_CanBusPeak)

find_package(PCan REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(CanBusPeak CanBusPeak.cpp
                           CanBusPeak.hpp
                           DeviceDriverImpl.cpp
                           ICanBusImpl.cpp
                           PeakCanMessage.cpp
                           PeakCanMessage.hpp)

add_dependencies(CanBusPeak COLOR_DEBUG)

target_link_libraries(CanBusPeak YARP::YARP_OS
                                 YARP::YARP_dev
                                 PCan::PCanFD
                                 YarpDevicesInterfaces)

yarp_install(TARGETS CanBusPeak
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CanBusPeak.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
