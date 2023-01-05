option(ENABLE_TrajectoryLib "Enable/disable TrajectoryLib library" ON)

if(ENABLE_TrajectoryLib)

    add_library(TrajectoryLib SHARED TrapezoidalTrajectory.hpp
                                     TrapezoidalTrajectory.cpp)

    set_property(TARGET TrajectoryLib PROPERTY PUBLIC_HEADER TrapezoidalTrajectory.hpp)

    target_include_directories(TrajectoryLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_compile_features(TrajectoryLib PUBLIC cxx_std_17)

    install(TARGETS TrajectoryLib
            EXPORT ROBOTICSLAB_YARP_DEVICES)

    add_library(ROBOTICSLAB::TrajectoryLib ALIAS TrajectoryLib)

else()

    set(ENABLE_TrajectoryLib OFF CACHE BOOL "Enable/disable TrajectoryLib library" FORCE)

endif()
