if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)
    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/googletest)
    include_directories(${GTestSources_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

    # FutureObserverLib

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/FutureObserverLib.cpp "#include \"FutureObserverLib.hpp\"")
    add_library(FutureObserverLib STATIC FutureObserverLib.hpp
                                         ${CMAKE_CURRENT_BINARY_DIR}/FutureObserverLib.cpp)
    target_compile_features(FutureObserverLib PUBLIC cxx_std_17)

    # testCanBusSharerLib

    if(ENABLE_CanBusSharerLib)
        add_executable(testCanBusSharerLib testCanBusSharerLib.cpp)
        target_link_libraries(testCanBusSharerLib ROBOTICSLAB::CanBusSharerLib gtest_main)
        gtest_discover_tests(testCanBusSharerLib)
    endif()

    # testStateObserverLib

    if(ENABLE_StateObserverLib)
        add_executable(testStateObserverLib testStateObserverLib.cpp)
        target_link_libraries(testStateObserverLib ROBOTICSLAB::StateObserverLib FutureObserverLib gtest_main)
        gtest_discover_tests(testStateObserverLib)
    endif()

    # testCanOpenNodeLib

    if(ENABLE_CanOpenNodeLib)
        add_executable(testCanOpenNodeLib testCanOpenNodeLib.cpp)
        target_link_libraries(testCanOpenNodeLib ROBOTICSLAB::CanOpenNodeLib FutureObserverLib gtest_main)
        gtest_discover_tests(testCanOpenNodeLib)
    endif()

    # testTrajectoryLib

    if(ENABLE_TrajectoryLib)
        add_executable(testTrajectoryLib testTrajectoryLib.cpp)
        target_link_libraries(testTrajectoryLib ROBOTICSLAB::TrajectoryLib gtest_main)
        gtest_discover_tests(testTrajectoryLib)
    endif()

    # testYarpDeviceMapperLib

    if(ENABLE_YarpDeviceMapperLib)
        add_executable(testYarpDeviceMapperLib testYarpDeviceMapperLib.cpp)
        target_link_libraries(testYarpDeviceMapperLib ROBOTICSLAB::YarpDeviceMapperLib YARP::YARP_dev gtest_main)
        gtest_discover_tests(testYarpDeviceMapperLib)
    endif()
else()
    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)
endif()
