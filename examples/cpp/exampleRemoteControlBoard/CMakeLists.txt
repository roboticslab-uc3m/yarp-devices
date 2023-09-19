cmake_minimum_required(VERSION 3.16)

project(exampleRemoteControlBoard LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.7 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleRemoteControlBoard exampleRemoteControlBoard.cpp)

target_link_libraries(exampleRemoteControlBoard YARP::YARP_os
                                                YARP::YARP_init
                                                YARP::YARP_dev)
