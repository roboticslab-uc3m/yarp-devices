option(ENABLE_dumpCanBus "Enable/disable dumpCanBus program" ON)

if(ENABLE_dumpCanBus)

    add_executable(dumpCanBus main.cpp
                              DumpCanBus.cpp
                              DumpCanBus.hpp)

    target_link_libraries(dumpCanBus YARP::YARP_OS
                                     YARP::YARP_init
                                     ROBOTICSLAB::ColorDebug)

    target_compile_features(dumpCanBus PRIVATE cxx_std_11)

    install(TARGETS dumpCanBus
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
