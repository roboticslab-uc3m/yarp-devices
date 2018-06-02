# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraControlboard
                    INCLUDE DextraControlboard.hpp
                    DEFAULT ON)

if(NOT SKIP_DextraControlboard)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(DextraControlboard DextraControlboard.cpp
                           DextraControlboard.hpp
                           DeviceDriverImpl.cpp
                           IControlLimits2Impl.cpp
                           IControlMode2Impl.cpp
                           IEncodersImpl.cpp
                           IEncodersTimedImpl.cpp
                           IInteractionModeImpl.cpp
                           IPositionControl2Impl.cpp
                           IPositionDirectImpl.cpp
                           ITorqueControlImpl.cpp
                           IVelocityControl2Impl.cpp)

add_dependencies(DextraControlboard COLOR_DEBUG)

target_link_libraries(DextraControlboard YARP::YARP_OS
                                 YARP::YARP_dev
                                 YarpDevicesInterfaces)

# Hack, fool CMake to think that YARP already switched to 3.0 while still at devel.
# https://github.com/roboticslab-uc3m/yarp-devices/issues/180
if(NOT YARP_VERSION_SHORT VERSION_LESS 2.3.73)
    target_compile_definitions(DextraControlboard PUBLIC YARP_VERSION_MAJOR=3)
endif()

yarp_install(TARGETS DextraControlboard
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES DextraControlboard.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
