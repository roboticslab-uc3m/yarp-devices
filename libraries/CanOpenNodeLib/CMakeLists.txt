cmake_dependent_option(ENABLE_CanOpenNodeLib "Enable/disable CanOpenNodeLib library" ON
                       ENABLE_CanBusSharerLib OFF)

if(ENABLE_CanOpenNodeLib)

    include(CheckIncludeFileCXX)
    check_include_file_cxx("optional" _has_optional)

    add_library(CanOpenNodeLib SHARED CanOpenNode.hpp
                                      CanOpenNode.cpp
                                      SdoClient.hpp
                                      SdoClient.cpp
                                      PdoProtocol.hpp
                                      PdoProtocol.cpp
                                      EmcyConsumer.hpp
                                      EmcyConsumer.cpp
                                      NmtProtocol.hpp
                                      NmtProtocol.cpp
                                      DriveStatusMachine.hpp
                                      DriveStatusMachine.cpp)

    set_property(TARGET CanOpenNodeLib PROPERTY PUBLIC_HEADER CanOpenNode.hpp
                                                              SdoClient.hpp
                                                              PdoProtocol.hpp
                                                              EmcyConsumer.hpp
                                                              NmtProtocol.hpp
                                                              DriveStatusMachine.hpp)

    if(_has_optional AND _idx_cxx_std_17 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
        set_source_files_properties(PdoProtocol.cpp PROPERTIES COMPILE_OPTIONS "-std=c++17")
    else()
        target_sources(CanOpenNodeLib PRIVATE nonstd/optional.hpp)
        set_source_files_properties(PdoProtocol.cpp PROPERTIES COMPILE_DEFINITIONS
            "USE_NONSTD_OPTIONAL;optional_CONFIG_NO_EXCEPTIONS=1")
    endif()

    target_link_libraries(CanOpenNodeLib PUBLIC ROBOTICSLAB::CanBusSharerLib
                                                ROBOTICSLAB::StateObserverLib
                                         PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(CanOpenNodeLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_compile_features(CanOpenNodeLib PUBLIC cxx_std_14)

    install(TARGETS CanOpenNodeLib
            EXPORT ROBOTICSLAB_YARP_DEVICES
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    add_library(ROBOTICSLAB::CanOpenNodeLib ALIAS CanOpenNodeLib)

else()

    set(ENABLE_CanOpenNodeLib OFF CACHE BOOL "Enable/disable CanOpenNodeLib library" FORCE)

endif()
