cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(examplePositionDirect LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
endif()

find_package(COLOR_DEBUG REQUIRED)

add_executable(examplePositionDirect examplePositionDirect.cpp)

target_link_libraries(examplePositionDirect YARP::YARP_os
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            ROBOTICSLAB::ColorDebug)
