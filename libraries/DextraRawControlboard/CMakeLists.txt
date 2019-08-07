option(ENABLE_DextraRawControlboardLib "Enable/disable DextraRawControlboardLib library" ON)

if(ENABLE_DextraRawControlboardLib)

    add_library(DextraRawControlboardLib SHARED DextraRawControlboard.hpp
                                                DextraRawControlboard.cpp
                                                IAxisInfoRawImpl.cpp
                                                IControlLimitsRawImpl.cpp
                                                IControlModeRawImpl.cpp
                                                IEncodersTimedRawImpl.cpp
                                                IPositionControlRawImpl.cpp
                                                IPositionDirectRawImpl.cpp)

    target_link_libraries(DextraRawControlboardLib PUBLIC YARP::YARP_OS
                                                          YARP::YARP_dev
                                                   PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(DextraRawControlboardLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    install(TARGETS DextraRawControlboardLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_DextraRawControlboardLib OFF CACHE BOOL "Enable/disable DextraRawControlboardLib library" FORCE)

endif()
