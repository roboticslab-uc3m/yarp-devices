option(ENABLE_launchManipulation "Enable/disable launchManipulation program" ON)

if(ENABLE_launchManipulation)

find_package(YARP REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

add_executable(launchManipulation main.cpp)

add_dependencies(launchManipulation COLOR_DEBUG)

target_link_libraries(launchManipulation ${YARP_LIBRARIES}
                                         TwoCanBusThreeWrappers)

install(TARGETS launchManipulation
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
