option(ENABLE_YarpDeviceMapperLib "Enable/disable YarpDeviceMapperLib library" ON)

if(ENABLE_YarpDeviceMapperLib)

    add_library(YarpDeviceMapperLib SHARED DeviceMapper.hpp
                                           DeviceMapper.cpp
                                           RawDevice.cpp
                                           FutureTask.hpp
                                           FutureTask.cpp
                                           ctpl/ctpl_stl.h)

    target_link_libraries(YarpDeviceMapperLib PUBLIC YARP::YARP_dev
                                              PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(YarpDeviceMapperLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    target_compile_features(YarpDeviceMapperLib PUBLIC cxx_std_14)

    install(TARGETS YarpDeviceMapperLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_YarpDeviceMapperLib OFF CACHE BOOL "Enable/disable YarpDeviceMapperLib library" FORCE)

endif()
