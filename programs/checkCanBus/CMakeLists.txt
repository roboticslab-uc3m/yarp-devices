cmake_dependent_option(ENABLE_checkCanBus "Enable/disable checkCanBus program" ON
                       ENABLE_YarpPlugins OFF)

if(ENABLE_checkCanBus)

find_package(YARP REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

add_executable(checkCanBus main.cpp
                           CheckCanBus.cpp
                           CheckCanBus.hpp
                           ThreadImpl.cpp)

add_dependencies(checkCanBus COLOR_DEBUG)

target_link_libraries(checkCanBus ${YARP_LIBRARIES}
                                  ${ROBOTICSLAB_YARP_DEVICES_LIBRARIES})

install(TARGETS checkCanBus
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
