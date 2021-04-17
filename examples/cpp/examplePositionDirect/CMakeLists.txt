cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(examplePositionDirect LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::StateObserverLib)
    find_package(ROBOTICSLAB_YARP_DEVICES)
endif()

add_executable(examplePositionDirectOfflineAsync examplePositionDirectOfflineAsync.cpp)
target_link_libraries(examplePositionDirectOfflineAsync ${YARP_LIBRARIES})

add_executable(examplePositionDirectOfflineSync examplePositionDirectOfflineSync.cpp)
target_link_libraries(examplePositionDirectOfflineSync ${YARP_LIBRARIES})

if(TARGET ROBOTICSLAB::StateObserverLib)
    add_executable(examplePositionDirectOnlinePullLocal examplePositionDirectOnlinePullLocal.cpp)
    target_link_libraries(examplePositionDirectOnlinePullLocal ${YARP_LIBRARIES} ROBOTICSLAB::StateObserverLib)
endif()

add_executable(examplePositionDirectOnlinePullRemote examplePositionDirectOnlinePullRemote.cpp)
target_link_libraries(examplePositionDirectOnlinePullRemote ${YARP_LIBRARIES})

add_executable(examplePositionDirectOnlinePush examplePositionDirectOnlinePush.cpp)
target_link_libraries(examplePositionDirectOnlinePush ${YARP_LIBRARIES})
