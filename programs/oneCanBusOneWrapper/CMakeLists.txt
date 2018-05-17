cmake_dependent_option(ENABLE_oneCanBusOneWrapper "Enable/disable oneCanBusOneWrapper program" ON
                       ENABLE_OneCanBusOneWrapper OFF)

if(ENABLE_oneCanBusOneWrapper)

add_executable(oneCanBusOneWrapper main.cpp)

add_dependencies(oneCanBusOneWrapper COLOR_DEBUG)

target_link_libraries(oneCanBusOneWrapper YARP::YARP_OS
                                          YARP::YARP_init
                                          OneCanBusOneWrapper)

install(TARGETS oneCanBusOneWrapper
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
