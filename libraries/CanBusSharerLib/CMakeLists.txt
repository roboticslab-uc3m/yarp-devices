option(ENABLE_CanBusSharerLib "Enable/disable CanBusSharerLib library" ON)

if(ENABLE_CanBusSharerLib)

    add_library(CanBusSharerLib SHARED ICanBusSharer.hpp
                                       CanOpen.hpp
                                       CanOpen.cpp
                                       CanSenderDelegate.hpp
                                       CanSenderDelegate.cpp
                                       SdoClient.hpp
                                       SdoClient-inl.hpp
                                       SdoClient.cpp
                                       PdoProtocol.hpp
                                       PdoProtocol.cpp
                                       EmcyConsumer.hpp
                                       EmcyConsumer.cpp
                                       NmtProtocol.hpp
                                       NmtProtocol.cpp
                                       DriveStatusMachine.hpp
                                       DriveStatusMachine.hpp
                                       StateObserver.hpp
                                       StateObserver.cpp
                                       CanUtils.hpp
                                       CanUtils.cpp)

    target_link_libraries(CanBusSharerLib PUBLIC YARP::YARP_dev
                                          PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(CanBusSharerLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    target_compile_definitions(CanBusSharerLib PRIVATE optional_CONFIG_NO_EXCEPTIONS=1)

    install(TARGETS CanBusSharerLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_CanBusSharerLib OFF CACHE BOOL "Enable/disable CanBusSharerLib library" FORCE)

endif()
