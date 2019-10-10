option(ENABLE_CanBusSharerLib "Enable/disable CanBusSharerLib library" ON)

if(ENABLE_CanBusSharerLib)

    add_library(CanBusSharerLib SHARED ICanBusSharer.hpp
                                       CanOpen.hpp
                                       CanOpen.cpp
                                       CanSenderDelegate.hpp
                                       SdoClient.hpp
                                       SdoClient.cpp
                                       PdoProtocol.hpp
                                       PdoProtocol.cpp
                                       EmcyConsumer.hpp
                                       EmcyConsumer.cpp
                                       NmtProtocol.hpp
                                       NmtProtocol.cpp
                                       DriveStatusMachine.hpp
                                       DriveStatusMachine.cpp
                                       StateObserver.hpp
                                       StateObserver.cpp
                                       CanUtils.hpp
                                       CanUtils.cpp)

    target_link_libraries(CanBusSharerLib PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(CanBusSharerLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    target_compile_definitions(CanBusSharerLib PRIVATE optional_CONFIG_NO_EXCEPTIONS=1)

    if(NOT CMAKE_VERSION VERSION_LESS 3.8)
        target_compile_features(CanBusSharerLib PUBLIC cxx_std_11)
    else()
        target_compile_features(CanBusSharerLib PUBLIC cxx_constexpr) # I'm too lazy
    endif()

    install(TARGETS CanBusSharerLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_CanBusSharerLib OFF CACHE BOOL "Enable/disable CanBusSharerLib library" FORCE)

endif()
