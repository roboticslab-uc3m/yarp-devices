option(ENABLE_CanBusSharerLib "Enable/disable CanBusSharerLib library" ON)

if(ENABLE_CanBusSharerLib)

    add_library(CanBusSharerLib SHARED ICanBusSharer.hpp
                                       CanSenderDelegate.hpp
                                       CanSenderDelegate.cpp
                                       CanBusSharer.cpp)

    target_link_libraries(CanBusSharerLib PUBLIC YARP::YARP_dev
                                          PRIVATE YARP::YARP_OS
                                                  ROBOTICSLAB::ColorDebug)

    target_include_directories(CanBusSharerLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    install(TARGETS CanBusSharerLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_CanBusSharerLib OFF CACHE BOOL "Enable/disable CanBusSharerLib library" FORCE)

endif()
