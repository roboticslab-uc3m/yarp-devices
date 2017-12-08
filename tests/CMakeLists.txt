option(ENABLE_tests "Choose if you want to compile tests" ON)

if(ENABLE_tests)

add_subdirectory(gtest-1.7.0)

enable_testing()

include_directories(${gtest_SOURCE_DIR}/include
                    ${gtest_SOURCE_DIR})

link_libraries(gtest
               gtest_main)

add_subdirectory(testTechnosoftIpos)

add_subdirectory(testCuiAbsolute)

endif()
