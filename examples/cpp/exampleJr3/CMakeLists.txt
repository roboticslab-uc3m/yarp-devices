cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

project(exampleJr3)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(exampleJr3 exampleJr3.cpp)
  
target_link_libraries(exampleJr3 YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 YARP::YARP_sig)

include(GNUInstallDirs)

install(TARGETS exampleJr3
        DESTINATION ${CMAKE_INSTALL_BINDIR})
