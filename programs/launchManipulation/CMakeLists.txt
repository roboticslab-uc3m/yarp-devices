cmake_dependent_option(ENABLE_launchManipulation "Enable/disable launchManipulation program" ON
                       ENABLE_TwoCanBusThreeWrappers OFF)

if(ENABLE_launchManipulation)

add_executable(launchManipulation main.cpp)

target_link_libraries(launchManipulation YARP::YARP_OS
                                         YARP::YARP_init
                                         ROBOTICSLAB::ColorDebug
                                         TwoCanBusThreeWrappers)

install(TARGETS launchManipulation
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
