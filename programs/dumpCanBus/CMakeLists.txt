option(ENABLE_dumpCanBus "Enable/disable dumpCanBus program" ON)

if(ENABLE_dumpCanBus)

    add_executable(dumpCanBus main.cpp
                              DumpCanBus.cpp
                              DumpCanBus.hpp)

    target_link_libraries(dumpCanBus YARP::YARP_os
                                     YARP::YARP_init)

    install(TARGETS dumpCanBus)

endif()
