# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TechnosoftIpos
                    CATEGORY device
                    TYPE roboticslab::TechnosoftIpos
                    INCLUDE TechnosoftIpos.hpp
                    DEFAULT ON)

if(NOT SKIP_TechnosoftIpos)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs})

yarp_add_plugin(TechnosoftIpos TechnosoftIpos.cpp
                               TechnosoftIpos.hpp
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

add_dependencies(TechnosoftIpos COLOR_DEBUG)

target_link_libraries(TechnosoftIpos YARP::YARP_OS
                                     YARP::YARP_dev)

yarp_install(TARGETS TechnosoftIpos
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES TechnosoftIpos.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
