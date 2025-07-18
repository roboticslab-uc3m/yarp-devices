cmake_minimum_required(VERSION 3.19...3.28)

project(exampleTrajectory LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::StateObserverLib)
    find_package(ROBOTICSLAB_YARP_DEVICES)
endif()

add_executable(exampleOfflineTrajectoryAsync exampleOfflineTrajectoryAsync.cpp)
target_link_libraries(exampleOfflineTrajectoryAsync ${YARP_LIBRARIES})

add_executable(exampleOfflineTrajectorySync exampleOfflineTrajectorySync.cpp)
target_link_libraries(exampleOfflineTrajectorySync ${YARP_LIBRARIES})

if(TARGET ROBOTICSLAB::StateObserverLib)
    add_executable(exampleOnlineTrajectoryLocalPull exampleOnlineTrajectoryLocalPull.cpp)
    target_link_libraries(exampleOnlineTrajectoryLocalPull ${YARP_LIBRARIES} ROBOTICSLAB::StateObserverLib)
endif()

add_executable(exampleOnlineTrajectoryRemotePull exampleOnlineTrajectoryRemotePull.cpp)
target_link_libraries(exampleOnlineTrajectoryRemotePull ${YARP_LIBRARIES})

add_executable(exampleOnlineTrajectoryRemotePush exampleOnlineTrajectoryRemotePush.cpp)
target_link_libraries(exampleOnlineTrajectoryRemotePush ${YARP_LIBRARIES})
