cmake_minimum_required(VERSION 3.19...3.28)

project(exampleRemoteJr3 LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleRemoteJr3 exampleRemoteJr3.cpp)

target_link_libraries(exampleRemoteJr3 YARP::YARP_os
                                       YARP::YARP_init
                                       YARP::YARP_dev)
