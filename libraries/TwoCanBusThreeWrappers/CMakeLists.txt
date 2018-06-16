option(ENABLE_TwoCanBusThreeWrappers "Enable/disable TwoCanBusThreeWrappers library" ON)

if(ENABLE_TwoCanBusThreeWrappers)

add_library(TwoCanBusThreeWrappers SHARED TwoCanBusThreeWrappers.cpp
                                          TwoCanBusThreeWrappers.hpp)

set_target_properties(TwoCanBusThreeWrappers PROPERTIES PUBLIC_HEADER TwoCanBusThreeWrappers.hpp)

target_link_libraries(TwoCanBusThreeWrappers PUBLIC YARP::YARP_OS
                                                    YARP::YARP_dev
                                             PRIVATE ROBOTICSLAB::ColorDebug)

target_include_directories(TwoCanBusThreeWrappers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS TwoCanBusThreeWrappers
        EXPORT ROBOTICSLAB_YARP_DEVICES
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

endif()
