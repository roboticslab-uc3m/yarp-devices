cmake_minimum_required(VERSION 3.16)

project(exampleJr3 LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.6 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleJr3 exampleJr3.cpp)

target_link_libraries(exampleJr3 YARP::YARP_os
                                 YARP::YARP_dev)
