# Testing things #########################################################################################

enable_testing()

find_package(YARP REQUIRED)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${YARP_DEVICES_INCLUDE_DIRS})
link_directories(${TEO_LINK_DIRS})

add_executable(testPlayback testPlayback.cpp)
target_link_libraries(testPlayback ${YARP_LIBRARIES} ${YARP_DEVICES_LIBRARIES} gtest gtest_main)

