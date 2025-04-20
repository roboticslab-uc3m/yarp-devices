cmake_minimum_required(VERSION 3.19...3.28)

project(exampleRemoteControlBoardModule LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.10 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleRemoteControlBoardModule main.cpp
                                               ExampleRemoteControlBoard.cpp
                                               ExampleRemoteControlBoard.hpp)

target_link_libraries(exampleRemoteControlBoardModule YARP::YARP_os
                                                      YARP::YARP_init
                                                      YARP::YARP_dev)
