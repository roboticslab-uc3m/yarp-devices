cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleCanBusControlboard LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS OS dev)
find_package(COLOR_DEBUG REQUIRED)

add_executable(exampleCanBusControlboard main.cpp
                                         ExampleCanBusControlboard.cpp
                                         ExampleCanBusControlboard.hpp)

target_link_libraries(exampleCanBusControlboard YARP::YARP_OS
                                                YARP::YARP_init
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug)

include(GNUInstallDirs)

install(TARGETS exampleCanBusControlboard
        DESTINATION ${CMAKE_INSTALL_BINDIR})
