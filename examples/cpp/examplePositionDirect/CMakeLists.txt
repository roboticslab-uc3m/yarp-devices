cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(examplePositionDirect LANGUAGES CXX)

find_package(COLOR_DEBUG REQUIRED)
find_package(YARP 3.2 REQUIRED COMPONENTS os dev)

add_executable(examplePositionDirect examplePositionDirect.cpp)

target_link_libraries(examplePositionDirect YARP::YARP_os
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            ROBOTICSLAB::ColorDebug)
