cmake_minimum_required(VERSION 3.19...3.28)

project(exampleRemoteControlBoard LANGUAGES CXX)

if(NOT YARP_FOUND)
    set(_yarp_components os dev)
    find_package(YARP 3.12 QUIET COMPONENTS ${_yarp_components})

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS ${_yarp_components})
    endif()
endif()

add_executable(exampleRemoteControlBoard exampleRemoteControlBoard.cpp)

target_link_libraries(exampleRemoteControlBoard YARP::YARP_os
                                                YARP::YARP_init
                                                YARP::YARP_dev)
