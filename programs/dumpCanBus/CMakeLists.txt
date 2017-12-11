cmake_dependent_option(ENABLE_dumpCanBus "Enable/disable dumpCanBus program" ON
                       ENABLE_YarpPlugins OFF)

if(ENABLE_dumpCanBus)

find_package(YARP REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

add_executable(dumpCanBus main.cpp
                          DumpCanBus.cpp
                          DumpCanBus.hpp
                          ThreadImpl.cpp)

add_dependencies(dumpCanBus COLOR_DEBUG)

target_link_libraries(dumpCanBus YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev)

install(TARGETS dumpCanBus
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
