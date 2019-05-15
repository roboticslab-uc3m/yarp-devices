cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(exampleRemoteControlboard LANGUAGES CXX)

find_package(YARP 3.0 REQUIRED COMPONENTS OS dev)

add_executable(exampleRemoteControlboard exampleRemoteControlboard.cpp)

target_link_libraries(exampleRemoteControlboard YARP::YARP_OS
                                                YARP::YARP_init
                                                YARP::YARP_dev)

