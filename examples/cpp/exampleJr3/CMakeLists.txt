cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleJr3 LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev sig)
endif()

add_executable(exampleJr3 exampleJr3.cpp)

target_link_libraries(exampleJr3 YARP::YARP_os
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 YARP::YARP_sig)
