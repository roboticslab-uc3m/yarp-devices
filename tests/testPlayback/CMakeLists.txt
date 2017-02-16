# Testing things #########################################################################################

enable_testing()

find_package(YARP REQUIRED)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${TEO_BODY_INCLUDE_DIRS})
link_directories(${TEO_LINK_DIRS})

add_executable(testPlayback testPlayback.cpp)
target_link_libraries(testPlayback ${YARP_LIBRARIES} ${TEO_BODY_LIBRARIES} gtest gtest_main)

