cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(exampleTechnosoftIpos)

find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(exampleTechnosoftIpos exampleTechnosoftIpos.cpp)

target_link_libraries(exampleTechnosoftIpos YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            ROBOTICSLAB::YarpDevicesInterfaces)

include(GNUInstallDirs)

install(TARGETS exampleTechnosoftIpos
        DESTINATION ${CMAKE_INSTALL_BINDIR})
