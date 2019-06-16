cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(examplePositionDirect LANGUAGES CXX)

find_package(YARP 3.1 REQUIRED COMPONENTS OS dev)

add_executable(examplePositionDirect examplePositionDirect.cpp)

target_link_libraries(examplePositionDirect YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev)
