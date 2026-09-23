cmake_minimum_required(VERSION 3.19...3.28)

project(exampleJr3Pci LANGUAGES CXX)

if(NOT YARP_FOUND)
    set(_yarp_components os dev)
    find_package(YARP 3.12 QUIET COMPONENTS ${_yarp_components})

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS ${_yarp_components})
    endif()
endif()

add_executable(exampleJr3Pci exampleJr3Pci.cpp)

target_link_libraries(exampleJr3Pci YARP::YARP_os
                                    YARP::YARP_dev)
