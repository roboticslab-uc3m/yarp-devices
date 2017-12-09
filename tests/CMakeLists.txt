cmake_dependent_option(ENABLE_tests "Compile unit tests" ON
                       ENABLE_YarpPlugins OFF)

if(ENABLE_tests)

add_subdirectory(gtest-1.7.0)

enable_testing()

include_directories(${gtest_SOURCE_DIR}/include
                    ${gtest_SOURCE_DIR})

add_subdirectory(testTechnosoftIpos)

add_subdirectory(testCuiAbsolute)

endif()
