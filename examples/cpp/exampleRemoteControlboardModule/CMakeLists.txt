cmake_minimum_required(VERSION 3.16)

project(exampleRemoteControlboardModule LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.6 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleRemoteControlboardModule main.cpp
                                               ExampleRemoteControlboard.cpp
                                               ExampleRemoteControlboard.hpp)

target_link_libraries(exampleRemoteControlboardModule YARP::YARP_os
                                                      YARP::YARP_init
                                                      YARP::YARP_dev)
