add_executable(testTechnosoftIpos testTechnosoftIpos.cpp)

target_link_libraries(testTechnosoftIpos YARP::YARP_OS
                                         YARP::YARP_dev
                                         ROBOTICSLAB::ColorDebug
                                         YarpDevicesInterfaces
                                         gtest
                                         gtest_main)
