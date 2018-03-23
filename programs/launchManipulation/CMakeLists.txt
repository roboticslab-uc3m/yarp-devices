cmake_dependent_option(ENABLE_launchManipulation "Enable/disable launchManipulation program" ON
                       ENABLE_TwoCanBusThreeWrappers OFF)

if(ENABLE_launchManipulation)

include_directories(${YARP_INCLUDE_DIRS}) # remove if YARP > v2.3.70

add_executable(launchManipulation main.cpp)

add_dependencies(launchManipulation COLOR_DEBUG)

target_link_libraries(launchManipulation YARP::YARP_OS
                                         YARP::YARP_init
                                         TwoCanBusThreeWrappers)

install(TARGETS launchManipulation
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
