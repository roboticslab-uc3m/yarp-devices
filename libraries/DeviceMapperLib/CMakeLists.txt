option(ENABLE_DeviceMapperLib "Enable/disable DeviceMapperLib library" ON)

if(ENABLE_DeviceMapperLib)

    add_library(DeviceMapperLib SHARED DeviceMapper.hpp
                                       DeviceMapper.cpp
                                       RawDevice.cpp)

    target_link_libraries(DeviceMapperLib PUBLIC YARP::YARP_dev
                                          PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(DeviceMapperLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    if(NOT CMAKE_VERSION VERSION_LESS 3.8)
        target_compile_features(DeviceMapperLib PUBLIC cxx_std_11)
    else()
        target_compile_features(DeviceMapperLib PUBLIC cxx_constexpr) # I'm too lazy
    endif()

    install(TARGETS DeviceMapperLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_DeviceMapperLib OFF CACHE BOOL "Enable/disable DeviceMapperLib library" FORCE)

endif()
