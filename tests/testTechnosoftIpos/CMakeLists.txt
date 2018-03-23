include_directories(${YARP_INCLUDE_DIRS}) # remove if YARP > v2.3.70

add_executable(testTechnosoftIpos testTechnosoftIpos.cpp)

add_dependencies(testTechnosoftIpos COLOR_DEBUG)

target_link_libraries(testTechnosoftIpos YARP::YARP_OS
                                         YARP::YARP_dev
                                         YarpDevicesInterfaces
                                         gtest
                                         gtest_main)
