find_package(YARP REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

add_executable(testCuiAbsolute testCuiAbsolute.cpp)

target_link_libraries(testCuiAbsolute YARP::YARP_OS
                                      YARP::YARP_dev
                                      gtest
                                      gtest_main)
