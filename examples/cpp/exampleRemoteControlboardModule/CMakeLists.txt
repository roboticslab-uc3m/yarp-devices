cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleRemoteControlboard LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS OS dev)

add_executable(exampleRemoteControlboard main.cpp
                                         ExampleRemoteControlboard.cpp
                                         ExampleRemoteControlboard.hpp)

target_link_libraries(exampleRemoteControlboard YARP::YARP_OS
                                                YARP::YARP_init
                                                YARP::YARP_dev)

include(GNUInstallDirs)

install(TARGETS exampleRemoteControlboard
        DESTINATION ${CMAKE_INSTALL_BINDIR})
