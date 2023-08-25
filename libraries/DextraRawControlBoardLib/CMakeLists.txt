option(ENABLE_DextraRawControlBoardLib "Enable/disable DextraRawControlBoardLib library" ON)

if(ENABLE_DextraRawControlBoardLib)

    add_library(DextraRawControlBoardLib SHARED DextraRawControlBoard.hpp
                                                DextraRawControlBoard.cpp
                                                DeviceDriverImpl.cpp
                                                IAxisInfoRawImpl.cpp
                                                IControlLimitsRawImpl.cpp
                                                IControlModeRawImpl.cpp
                                                IEncodersRawImpl.cpp
                                                IPositionControlRawImpl.cpp
                                                IPositionDirectRawImpl.cpp
                                                LogComponent.hpp
                                                LogComponent.cpp)

    target_link_libraries(DextraRawControlBoardLib PUBLIC YARP::YARP_dev
                                                   PRIVATE YARP::YARP_os)

    target_include_directories(DextraRawControlBoardLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    install(TARGETS DextraRawControlBoardLib)

    add_library(ROBOTICSLAB::DextraRawControlBoardLib ALIAS DextraRawControlBoardLib)

else()

    set(ENABLE_DextraRawControlBoardLib OFF CACHE BOOL "Enable/disable DextraRawControlBoardLib library" FORCE)

endif()
