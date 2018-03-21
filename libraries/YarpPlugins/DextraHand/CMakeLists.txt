# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraHand
                    CATEGORY device
                    TYPE roboticslab::DextraHand
                    INCLUDE DextraHand.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_DextraHand)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(DextraHand DextraHand.cpp
                             DextraHand.hpp
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

add_dependencies(DextraHand COLOR_DEBUG)

target_link_libraries(DextraHand YARP::YARP_OS
                                   YARP::YARP_dev
                                   YarpDevicesInterfaces)

yarp_install(TARGETS DextraHand
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES DextraHand.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
