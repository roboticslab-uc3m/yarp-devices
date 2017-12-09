option(ENABLE_OneCanBusOneWrapper "Enable/disable OneCanBusOneWrapper library" ON)

if(ENABLE_OneCanBusOneWrapper)

find_package(YARP REQUIRED)

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

add_library(OneCanBusOneWrapper OneCanBusOneWrapper.cpp
                                OneCanBusOneWrapper.hpp)

add_dependencies(OneCanBusOneWrapper COLOR_DEBUG)

set_target_properties(OneCanBusOneWrapper PROPERTIES PUBLIC_HEADER OneCanBusOneWrapper.hpp)

target_link_libraries(OneCanBusOneWrapper LINK_PUBLIC YARP::YARP_OS
                                                      YARP::YARP_dev)

install(TARGETS OneCanBusOneWrapper
        EXPORT ROBOTICSLAB_YARP_DEVICES
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_YARP_DEVICES_TARGETS OneCanBusOneWrapper)

endif()
