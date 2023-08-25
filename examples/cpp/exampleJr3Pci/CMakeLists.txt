cmake_minimum_required(VERSION 3.16)

project(exampleJr3Pci LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.6 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleJr3Pci exampleJr3Pci.cpp)

target_link_libraries(exampleJr3Pci YARP::YARP_os
                                    YARP::YARP_dev)
