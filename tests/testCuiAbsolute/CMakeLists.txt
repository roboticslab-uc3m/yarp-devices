add_executable(testCuiAbsolute testCuiAbsolute.cpp)

target_link_libraries(testCuiAbsolute YARP::YARP_OS
                                      YARP::YARP_dev
                                      ROBOTICSLAB::ColorDebug
                                      YarpDevicesInterfaces
                                      gtest
                                      gtest_main)
