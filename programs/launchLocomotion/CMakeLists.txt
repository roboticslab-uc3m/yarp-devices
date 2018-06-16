cmake_dependent_option(ENABLE_launchLocomotion "Enable/disable launchLocomotion program" ON
                       ENABLE_TwoCanBusThreeWrappers OFF)

if(ENABLE_launchLocomotion)

add_executable(launchLocomotion main.cpp)

target_link_libraries(launchLocomotion YARP::YARP_OS
                                       YARP::YARP_init
                                       ROBOTICSLAB::ColorDebug
                                       TwoCanBusThreeWrappers)

install(TARGETS launchLocomotion
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
