cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(exampleRemoteJr3 LANGUAGES CXX)

find_package(YARP 3.0 REQUIRED COMPONENTS OS dev sig)

add_executable(exampleRemoteJr3 exampleRemoteJr3.cpp)
  
target_link_libraries(exampleRemoteJr3 YARP::YARP_OS
                                       YARP::YARP_init
                                       YARP::YARP_dev
                                       YARP::YARP_sig)

include(GNUInstallDirs)

install(TARGETS exampleRemoteJr3
        DESTINATION ${CMAKE_INSTALL_BINDIR})
