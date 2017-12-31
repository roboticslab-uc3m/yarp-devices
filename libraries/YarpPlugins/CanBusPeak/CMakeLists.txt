yarp_prepare_plugin(CanBusPeak
                    CATEGORY device
                    TYPE roboticslab::CanBusPeak
                    INCLUDE CanBusPeak.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;NOT CMAKE_VERSION VERSION_LESS 2.8.11")

if(NOT SKIP_CanBusPeak)

find_package(PCan REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs})

yarp_add_plugin(CanBusPeak CanBusPeak.cpp
                           CanBusPeak.hpp
                           DeviceDriverImpl.cpp
                           ICanBusImpl.cpp
                           PeakCanMessage.cpp
                           PeakCanMessage.hpp)

add_dependencies(CanBusPeak COLOR_DEBUG)

target_link_libraries(CanBusPeak YARP::YARP_OS
                                 YARP::YARP_dev
                                 PCan::PCanFD)

yarp_install(TARGETS CanBusPeak
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CanBusPeak.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
