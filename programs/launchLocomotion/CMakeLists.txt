cmake_dependent_option(ENABLE_launchLocomotion "Enable/disable launchLocomotion program" ON
                       ENABLE_TwoCanBusThreeWrappers OFF)

if(ENABLE_launchLocomotion)

find_package(YARP REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

add_executable(launchLocomotion main.cpp)

add_dependencies(launchLocomotion COLOR_DEBUG)

target_link_libraries(launchLocomotion YARP::YARP_OS
                                       YARP::YARP_init
                                       TwoCanBusThreeWrappers)

install(TARGETS launchLocomotion
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
