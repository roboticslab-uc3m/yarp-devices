cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

project(exampleRemoteJr3-2)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(exampleRemoteJr3-2 exampleRemoteJr3-2.cpp)
  
target_link_libraries(exampleRemoteJr3-2 YARP::YARP_OS
                                         YARP::YARP_init
                                         YARP::YARP_dev
                                         YARP::YARP_sig)

include(GNUInstallDirs)

install(TARGETS exampleRemoteJr3-2
        DESTINATION ${CMAKE_INSTALL_BINDIR})
