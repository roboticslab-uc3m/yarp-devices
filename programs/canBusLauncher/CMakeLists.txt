option(ENABLE_canBusLauncher "Enable/disable canBusLauncher program" ON)

if(ENABLE_canBusLauncher)

    add_executable(canBusLauncher main.cpp
                                  CanBusLauncher.hpp
                                  CanBusLauncher.cpp)
    
    target_link_libraries(canBusLauncher YARP::YARP_OS
                                         YARP::YARP_dev
                                         YARP::YARP_init
                                         ROBOTICSLAB::ColorDebug)
    
    install(TARGETS canBusLauncher
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
