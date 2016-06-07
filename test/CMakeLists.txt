# Testing things #########################################################################################

add_subdirectory(gtest-1.7.0)

add_subdirectory(testTechnosoftIpos)

add_subdirectory(testCuiAbsolute)

#enable_testing()

# find_package(YARP REQUIRED)

#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${TEO_BODY_INCLUDE_DIRS})
#link_directories(${TEO_LINK_DIRS})

# add_executable(testTechnosoftIpos testTechnosoftIpos.cpp)
#target_link_libraries(testTechnosoftIpos ${YARP_LIBRARIES} ${TEO_BODY_LIBRARIES} gtest gtest_main)

