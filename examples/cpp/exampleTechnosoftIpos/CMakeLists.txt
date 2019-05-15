cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(exampleTechnosoftIpos LANGUAGES CXX)

find_package(YARP 3.0 REQUIRED COMPONENTS OS dev)
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

add_executable(exampleTechnosoftIpos exampleTechnosoftIpos.cpp)

target_link_libraries(exampleTechnosoftIpos YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            ROBOTICSLAB::YarpDevicesInterfaces)

include(GNUInstallDirs)

install(TARGETS exampleTechnosoftIpos
        DESTINATION ${CMAKE_INSTALL_BINDIR})
