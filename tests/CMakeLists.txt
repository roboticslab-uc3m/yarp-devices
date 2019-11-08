if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)
    enable_testing()

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

    include_directories(${GTestSources_INCLUDE_DIR})

    # testCanBusSharerLib

    if(TARGET CanBusSharerLib)
        add_executable(testCanBusSharerLib testCanBusSharerLib.cpp)
        target_link_libraries(testCanBusSharerLib CanBusSharerLib gtest_main)
        gtest_add_tests(testCanBusSharerLib "" AUTO)
    endif()

    # testDeviceMapperLib

    if(TARGET DeviceMapperLib)
        add_executable(testDeviceMapperLib testDeviceMapperLib.cpp)
        target_link_libraries(testDeviceMapperLib DeviceMapperLib YARP::YARP_dev gtest_main)
        gtest_add_tests(testDeviceMapperLib "" AUTO)
    endif()

    #add_subdirectory(testTechnosoftIpos)
    #add_subdirectory(testCuiAbsolute)
else()
    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)
endif()
