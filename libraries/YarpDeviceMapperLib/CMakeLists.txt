option(ENABLE_YarpDeviceMapperLib "Enable/disable YarpDeviceMapperLib library" ON)

if(ENABLE_YarpDeviceMapperLib)

    add_library(YarpDeviceMapperLib SHARED DeviceMapper.hpp
                                           DeviceMapper.cpp
                                           RawDevice.cpp
                                           FutureTask.hpp
                                           FutureTask.cpp
                                           ctpl/ctpl_stl.h)

    target_link_libraries(YarpDeviceMapperLib PUBLIC YARP::YARP_dev
                                              PRIVATE YARP::YARP_os)

    target_include_directories(YarpDeviceMapperLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    install(TARGETS YarpDeviceMapperLib)

    add_library(ROBOTICSLAB::YarpDeviceMapperLib ALIAS YarpDeviceMapperLib)

else()

    set(ENABLE_YarpDeviceMapperLib OFF CACHE BOOL "Enable/disable YarpDeviceMapperLib library" FORCE)

endif()
