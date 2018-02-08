cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

project(exampleRemoteControlboard)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(exampleRemoteControlboard main.cpp
                                         ExampleRemoteControlboard.cpp
                                         ExampleRemoteControlboard.hpp)
  
target_link_libraries(exampleRemoteControlboard YARP::YARP_OS
                                                YARP::YARP_init
                                                YARP::YARP_dev)

include(GNUInstallDirs)

install(TARGETS exampleRemoteControlboard
        DESTINATION ${CMAKE_INSTALL_BINDIR}) 