option(ENABLE_checkCanBus "Enable/disable checkCanBus program" ON)

if(ENABLE_checkCanBus)

add_executable(checkCanBus main.cpp
                           CheckCanBus.cpp
                           CheckCanBus.hpp
                           ThreadImpl.cpp)

target_link_libraries(checkCanBus YARP::YARP_OS
                                  YARP::YARP_init
                                  YARP::YARP_dev
                                  ROBOTICSLAB::ColorDebug
                                  YarpDevicesInterfaces)

install(TARGETS checkCanBus
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
