cmake_dependent_option(ENABLE_CanOpenNodeLib "Enable/disable CanOpenNodeLib library" ON
                       ENABLE_CanBusSharerLib OFF)

if(ENABLE_CanOpenNodeLib)

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

    target_link_libraries(CanOpenNodeLib PUBLIC ROBOTICSLAB::CanBusSharerLib
                                                ROBOTICSLAB::StateObserverLib
                                         PRIVATE YARP::YARP_os)

    target_include_directories(CanOpenNodeLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_compile_features(CanOpenNodeLib PUBLIC cxx_std_17)

    install(TARGETS CanOpenNodeLib
            EXPORT ROBOTICSLAB_YARP_DEVICES)

    add_library(ROBOTICSLAB::CanOpenNodeLib ALIAS CanOpenNodeLib)

else()

    set(ENABLE_CanOpenNodeLib OFF CACHE BOOL "Enable/disable CanOpenNodeLib library" FORCE)

endif()
