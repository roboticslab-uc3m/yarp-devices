option(ENABLE_DextraRawControlboardLib "Enable/disable DextraRawControlboardLib library" ON)

if(ENABLE_DextraRawControlboardLib)

    add_library(DextraRawControlboardLib SHARED DextraRawControlboard.hpp
                                                DextraRawControlboard.cpp
                                                DeviceDriverImpl.cpp
                                                IAxisInfoRawImpl.cpp
                                                IControlLimitsRawImpl.cpp
                                                IControlModeRawImpl.cpp
                                                IEncodersRawImpl.cpp
                                                IPositionControlRawImpl.cpp
                                                IPositionDirectRawImpl.cpp
                                                LogComponent.hpp
                                                LogComponent.cpp)

    target_link_libraries(DextraRawControlboardLib PUBLIC YARP::YARP_dev
                                                   PRIVATE YARP::YARP_os)

    target_include_directories(DextraRawControlboardLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    install(TARGETS DextraRawControlboardLib)

    add_library(ROBOTICSLAB::DextraRawControlboardLib ALIAS DextraRawControlboardLib)

else()

    set(ENABLE_DextraRawControlboardLib OFF CACHE BOOL "Enable/disable DextraRawControlboardLib library" FORCE)

endif()
