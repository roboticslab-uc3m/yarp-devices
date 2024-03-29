option(ENABLE_CanBusSharerLib "Enable/disable CanBusSharerLib library" ON)

if(ENABLE_CanBusSharerLib)

    add_library(CanBusSharerLib SHARED ICanBusSharer.hpp
                                       ICanMessageNotifier.hpp
                                       ICanSenderDelegate.hpp
                                       CanMessage.hpp
                                       CanUtils.hpp
                                       CanUtils.cpp)

    set_property(TARGET CanBusSharerLib PROPERTY PUBLIC_HEADER ICanBusSharer.hpp
                                                               ICanMessageNotifier.hpp
                                                               ICanSenderDelegate.hpp
                                                               CanMessage.hpp
                                                               CanUtils.hpp)

    target_include_directories(CanBusSharerLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_compile_features(CanBusSharerLib PUBLIC cxx_std_17)

    install(TARGETS CanBusSharerLib
            EXPORT ROBOTICSLAB_YARP_DEVICES)

    add_library(ROBOTICSLAB::CanBusSharerLib ALIAS CanBusSharerLib)

else()

    set(ENABLE_CanBusSharerLib OFF CACHE BOOL "Enable/disable CanBusSharerLib library" FORCE)

endif()
