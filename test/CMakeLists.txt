# Testing things #########################################################################################
add_subdirectory(gtest-1.7.0)
enable_testing()

find_package(YARP REQUIRED)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${TEO_INCLUDE_DIRS})
link_directories(${TEO_LINK_DIRS})

add_executable(testTechnosoftIpos testTechnosoftIpos.cpp)
target_link_libraries(testTechnosoftIpos ${YARP_LIBRARIES} BodyYarp ${TEO_LIBRARIES} gtest gtest_main)

