# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(Spnav QUIET)

if(NOT Spnav_FOUND AND (NOT DEFINED ENABLE_SpaceNavigator OR ENABLE_SpaceNavigator))
    message(WARNING "Spnav package not found, disabling SpaceNavigator")
endif()

yarp_prepare_plugin(SpaceNavigator
                    CATEGORY device
                    TYPE roboticslab::SpaceNavigator
                    INCLUDE SpaceNavigator.hpp
                    DEFAULT ON
                    DEPENDS "UNIX;Spnav_FOUND")

if(NOT SKIP_SpaceNavigator)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                        ${SPNAV_INCLUDE_DIRS})

    yarp_add_plugin(SpaceNavigator SpaceNavigator.cpp
                                   SpaceNavigator.hpp
                                   DeviceDriverImpl.cpp
                                   IAnalogSensorImpl.cpp)

    add_dependencies(SpaceNavigator COLOR_DEBUG)

    target_link_libraries(SpaceNavigator YARP::YARP_dev
                                         ${SPNAV_LIBRARIES}
                                         YarpDevicesInterfaces)

    yarp_install(TARGETS SpaceNavigator
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES SpaceNavigator.ini
                 COMPONENT runtime
                 DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_SpaceNavigator OFF CACHE BOOL "Enable/disable SpaceNavigator device" FORCE)

endif()
