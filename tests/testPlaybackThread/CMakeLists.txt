# Testing things #########################################################################################

enable_testing()

find_package(YARP REQUIRED)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS})
link_directories(${ROBOTICSLAB_YARP_DEVICES_LINK_DIRS})

add_executable(testPlaybackThread testPlaybackThread.cpp)
target_link_libraries(testPlaybackThread ${YARP_LIBRARIES} ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES} gtest gtest_main)

