cmake_dependent_option(ENABLE_launchLocomotion "Enable/disable launchLocomotion program" ON
                       ENABLE_TwoCanBusThreeWrappers OFF)

if(ENABLE_launchLocomotion)

add_executable(launchLocomotion main.cpp)

add_dependencies(launchLocomotion COLOR_DEBUG)

target_link_libraries(launchLocomotion YARP::YARP_OS
                                       YARP::YARP_init
                                       TwoCanBusThreeWrappers)

install(TARGETS launchLocomotion
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
