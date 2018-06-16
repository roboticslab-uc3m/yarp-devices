option(ENABLE_OneCanBusOneWrapper "Enable/disable OneCanBusOneWrapper library" ON)

if(ENABLE_OneCanBusOneWrapper)

add_library(OneCanBusOneWrapper SHARED OneCanBusOneWrapper.cpp
                                       OneCanBusOneWrapper.hpp)

set_target_properties(OneCanBusOneWrapper PROPERTIES PUBLIC_HEADER OneCanBusOneWrapper.hpp)

target_link_libraries(OneCanBusOneWrapper PUBLIC YARP::YARP_OS
                                                 YARP::YARP_dev
                                          PRIVATE ROBOTICSLAB::ColorDebug)

target_include_directories(OneCanBusOneWrapper PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS OneCanBusOneWrapper
        EXPORT ROBOTICSLAB_YARP_DEVICES
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

endif()
