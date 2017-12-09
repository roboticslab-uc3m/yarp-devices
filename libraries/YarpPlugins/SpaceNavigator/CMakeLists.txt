# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(SpaceNavigator
                    CATEGORY device
                    TYPE roboticslab::SpaceNavigator
                    INCLUDE SpaceNavigator.hpp
                    DEFAULT ON)

if(NOT SKIP_SpaceNavigator)

find_package(Spnav REQUIRED)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs}
                    ${SPNAV_INCLUDE_DIRS})

yarp_add_plugin(SpaceNavigator SpaceNavigator.cpp
                               SpaceNavigator.hpp
                               DeviceDriverImpl.cpp
                               IAnalogSensorImpl.cpp)

add_dependencies(SpaceNavigator COLOR_DEBUG)

target_link_libraries(SpaceNavigator YARP::YARP_dev
                                     ${SPNAV_LIBRARIES})

yarp_install(TARGETS SpaceNavigator
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES SpaceNavigator.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
