cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleJr3 LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS OS dev sig)

add_executable(exampleJr3 exampleJr3.cpp)

target_link_libraries(exampleJr3 YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 YARP::YARP_sig)

include(GNUInstallDirs)

install(TARGETS exampleJr3
        DESTINATION ${CMAKE_INSTALL_BINDIR})
