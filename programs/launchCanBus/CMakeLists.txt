option(ENABLE_launchCanBus "Enable/disable launchCanBus program" ON)

if(ENABLE_launchCanBus)

    add_executable(launchCanBus main.cpp
                                LaunchCanBus.hpp
                                LaunchCanBus.cpp)
    
    target_link_libraries(launchCanBus YARP::YARP_os
                                       YARP::YARP_dev
                                       YARP::YARP_init)
    
    install(TARGETS launchCanBus
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
