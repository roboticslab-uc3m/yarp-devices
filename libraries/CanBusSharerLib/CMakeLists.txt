option(ENABLE_CanBusSharerLib "Enable/disable CanBusSharerLib library" ON)

if(ENABLE_CanBusSharerLib)

    include(CheckIncludeFileCXX)
    check_include_file_cxx("optional" _has_optional)

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

    if(_has_optional AND _idx_cxx_std_17 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
        set_source_files_properties(PdoProtocol.cpp PROPERTIES COMPILE_OPTIONS "-std=c++17")
    else()
        target_sources(CanBusSharerLib PRIVATE nonstd/optional.hpp)
        set_source_files_properties(PdoProtocol.cpp PROPERTIES COMPILE_DEFINITIONS
            "USE_NONSTD_OPTIONAL;optional_CONFIG_NO_EXCEPTIONS=1")
    endif()

    target_link_libraries(CanBusSharerLib PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(CanBusSharerLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    target_compile_features(CanBusSharerLib PUBLIC cxx_std_11)

    install(TARGETS CanBusSharerLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_CanBusSharerLib OFF CACHE BOOL "Enable/disable CanBusSharerLib library" FORCE)

endif()
