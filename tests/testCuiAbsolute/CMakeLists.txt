cmake_minimum_required(VERSION 2.6)

add_executable(testCuiAbsolute testCuiAbsolute.cpp)

find_package(YARP REQUIRED)
find_package(COLOR_DEBUG REQUIRED)
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${TEMPLATE_NAME_INCLUDE_DIRS}) 

target_link_libraries(testCuiAbsolute ${YARP_LIBRARIES}
                                      ${GTEST_BOTH_LIBRARIES}
                                      ROBOTICSLAB::ColorDebug
                                      ${CMAKE_THREAD_LIBS_INIT}
                                      )
