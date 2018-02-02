# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TextilesHand
                    CATEGORY device
                    TYPE roboticslab::TextilesHand
                    INCLUDE TextilesHand.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_TextilesHand)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs})

yarp_add_plugin(TextilesHand TextilesHand.cpp
                             TextilesHand.hpp
                             DeviceDriverImpl.cpp
                             ICanBusSharerImpl.cpp
                             IControlLimits2RawImpl.cpp
                             IControlMode2RawImpl.cpp
                             IEncodersRawImpl.cpp
                             IEncodersTimedRawImpl.cpp
                             IInteractionModeRawImpl.cpp
                             IPositionControl2RawImpl.cpp
                             IPositionDirectRawImpl.cpp
                             ITorqueControlRawImpl.cpp
                             IVelocityControl2RawImpl.cpp)

add_dependencies(TextilesHand COLOR_DEBUG)

target_link_libraries(TextilesHand YARP::YARP_OS
                                   YARP::YARP_dev)

yarp_install(TARGETS TextilesHand
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES TextilesHand.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
