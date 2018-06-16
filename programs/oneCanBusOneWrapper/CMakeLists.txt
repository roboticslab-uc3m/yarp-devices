cmake_dependent_option(ENABLE_oneCanBusOneWrapper "Enable/disable oneCanBusOneWrapper program" ON
                       ENABLE_OneCanBusOneWrapper OFF)

if(ENABLE_oneCanBusOneWrapper)

add_executable(oneCanBusOneWrapper main.cpp)

target_link_libraries(oneCanBusOneWrapper YARP::YARP_OS
                                          YARP::YARP_init
                                          ROBOTICSLAB::ColorDebug
                                          OneCanBusOneWrapper)

install(TARGETS oneCanBusOneWrapper
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
