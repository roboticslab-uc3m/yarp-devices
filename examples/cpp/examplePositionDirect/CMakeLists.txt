cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(examplePositionDirect LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
endif()

add_executable(examplePositionDirectOfflineAsync examplePositionDirectOfflineAsync.cpp)

target_link_libraries(examplePositionDirectOfflineAsync YARP::YARP_os
                                                        YARP::YARP_init
                                                        YARP::YARP_dev)

add_executable(examplePositionDirectOfflineSync examplePositionDirectOfflineSync.cpp)

target_link_libraries(examplePositionDirectOfflineSync YARP::YARP_os
                                                       YARP::YARP_init
                                                       YARP::YARP_dev)

add_executable(examplePositionDirectOnline examplePositionDirectOnline.cpp)

target_link_libraries(examplePositionDirectOnline YARP::YARP_os
                                                  YARP::YARP_init
                                                  YARP::YARP_dev)
