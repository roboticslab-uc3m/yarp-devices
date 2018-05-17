add_executable(testCuiAbsolute testCuiAbsolute.cpp)

add_dependencies(testCuiAbsolute COLOR_DEBUG)

target_link_libraries(testCuiAbsolute YARP::YARP_OS
                                      YARP::YARP_dev
                                      YarpDevicesInterfaces
                                      gtest
                                      gtest_main)
