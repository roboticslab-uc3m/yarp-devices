cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(exampleCanBusControlboard)

find_package(YARP REQUIRED)
find_package(COLOR_DEBUG REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

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