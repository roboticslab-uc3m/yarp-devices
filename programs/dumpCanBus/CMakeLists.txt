option(ENABLE_dumpCanBus "Enable/disable dumpCanBus program" ON)

if(ENABLE_dumpCanBus)

include_directories(${YARP_INCLUDE_DIRS}) # remove if YARP > v2.3.70

add_executable(dumpCanBus main.cpp
                          DumpCanBus.cpp
                          DumpCanBus.hpp
                          ThreadImpl.cpp)

add_dependencies(dumpCanBus COLOR_DEBUG)

target_link_libraries(dumpCanBus YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 YarpDevicesInterfaces)

install(TARGETS dumpCanBus
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
