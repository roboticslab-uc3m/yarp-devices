if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)
    enable_testing()
    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)
    include_directories(${GTestSources_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

    # FutureObserverLib

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/FutureObserverLib.cpp "#include \"FutureObserverLib.hpp\"")
    add_library(FutureObserverLib STATIC FutureObserverLib.hpp
                                         ${CMAKE_CURRENT_BINARY_DIR}/FutureObserverLib.cpp)
    target_compile_features(FutureObserverLib PUBLIC cxx_std_11)

    # testCanBusSharerLib

    if(TARGET CanBusSharerLib)
        add_executable(testCanBusSharerLib testCanBusSharerLib.cpp)
        target_link_libraries(testCanBusSharerLib CanBusSharerLib gtest_main)
        gtest_discover_tests(testCanBusSharerLib)
    endif()

    # testStateObserverLib

    if(TARGET StateObserverLib)
        add_executable(testStateObserverLib testStateObserverLib.cpp)
        target_link_libraries(testStateObserverLib StateObserverLib FutureObserverLib gtest_main)
        gtest_discover_tests(testStateObserverLib)
    endif()

    # testCanOpenNodeLib

    if(TARGET CanOpenNodeLib)
        add_executable(testCanOpenNodeLib testCanOpenNodeLib.cpp)
        target_link_libraries(testCanOpenNodeLib CanOpenNodeLib FutureObserverLib gtest_main)
        gtest_discover_tests(testCanOpenNodeLib)
    endif()

    # testYarpDeviceMapperLib

    if(TARGET YarpDeviceMapperLib)
        add_executable(testYarpDeviceMapperLib testYarpDeviceMapperLib.cpp)
        target_link_libraries(testYarpDeviceMapperLib YarpDeviceMapperLib YARP::YARP_dev gtest_main)
        gtest_discover_tests(testYarpDeviceMapperLib)
    endif()
else()
    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)
endif()
