cmake_dependent_option(ENABLE_dumpCanBus "Enable/disable dumpCanBus program" ON
                       ENABLE_CanBusSharerLib OFF)

if(ENABLE_dumpCanBus)

    add_executable(dumpCanBus main.cpp
                              DumpCanBus.cpp
                              DumpCanBus.hpp
                              ThreadImpl.cpp)

    target_link_libraries(dumpCanBus YARP::YARP_OS
                                     YARP::YARP_init
                                     YARP::YARP_dev
                                     CanBusSharerLib
                                     ROBOTICSLAB::ColorDebug
                                     YarpDevicesInterfaces)

    install(TARGETS dumpCanBus
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
