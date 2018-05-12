cmake_dependent_option(ENABLE_checkCanBus "Enable/disable checkCanBus program" ON
                       ENABLE_YarpPlugins OFF)

if(ENABLE_checkCanBus)

include_directories(${YARP_INCLUDE_DIRS}) # remove if YARP > v2.3.70

add_executable(checkCanBus main.cpp
                           CheckCanBus.cpp
                           CheckCanBus.hpp
                           ThreadImpl.cpp)

add_dependencies(checkCanBus COLOR_DEBUG)

target_link_libraries(checkCanBus YARP::YARP_OS
                                  YARP::YARP_init
                                  YARP::YARP_dev
                                  YarpDevicesInterfaces)

install(TARGETS checkCanBus
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
