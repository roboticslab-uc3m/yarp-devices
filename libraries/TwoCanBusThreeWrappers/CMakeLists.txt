option(ENABLE_TwoCanBusThreeWrappers "Enable/disable TwoCanBusThreeWrappers library" ON)

if(ENABLE_TwoCanBusThreeWrappers)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS}) # remove if YARP > v2.3.70

add_library(TwoCanBusThreeWrappers TwoCanBusThreeWrappers.cpp
                                   TwoCanBusThreeWrappers.hpp)

add_dependencies(TwoCanBusThreeWrappers COLOR_DEBUG)

set_target_properties(TwoCanBusThreeWrappers PROPERTIES PUBLIC_HEADER TwoCanBusThreeWrappers.hpp)

target_link_libraries(TwoCanBusThreeWrappers LINK_PUBLIC YARP::YARP_OS
                                                         YARP::YARP_dev)

install(TARGETS TwoCanBusThreeWrappers
        EXPORT ROBOTICSLAB_YARP_DEVICES
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_YARP_DEVICES_TARGETS TwoCanBusThreeWrappers)

endif()
