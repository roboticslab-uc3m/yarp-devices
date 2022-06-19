cmake_minimum_required(VERSION 3.16)

project(exampleRemoteControlboard LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.6 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleRemoteControlboard exampleRemoteControlboard.cpp)

target_link_libraries(exampleRemoteControlboard YARP::YARP_os
                                                YARP::YARP_init
                                                YARP::YARP_dev)
