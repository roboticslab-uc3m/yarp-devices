option(ENABLE_StateObserverLib "Enable/disable StateObserverLib library" ON)

if(ENABLE_StateObserverLib)

    add_library(StateObserverLib SHARED StateObserver.hpp
                                        StateObserver.cpp)

    set_target_properties(StateObserverLib PROPERTIES PUBLIC_HEADER StateObserver.hpp)

    target_include_directories(StateObserverLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_compile_features(StateObserverLib PUBLIC cxx_std_17)

    install(TARGETS StateObserverLib
            EXPORT ROBOTICSLAB_YARP_DEVICES)

    add_library(ROBOTICSLAB::StateObserverLib ALIAS StateObserverLib)

else()

    set(ENABLE_StateObserverLib OFF CACHE BOOL "Enable/disable StateObserverLib library" FORCE)

endif()
