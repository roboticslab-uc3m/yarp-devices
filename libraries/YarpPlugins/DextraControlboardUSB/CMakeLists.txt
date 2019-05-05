# Copyright: (C) 2019 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Jennifer Joana Gago Muñoz
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraControlboardUSB
                    CATEGORY device
                    TYPE roboticslab::DextraControlboardUSB
                    INCLUDE DextraControlboardUSB.hpp
                    DEFAULT ON
                    DEPENDS UNIX)

if(NOT SKIP_DextraControlboardUSB)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(DextraControlboardUSB DextraControlboardUSB.cpp
                                      DextraControlboardUSB.hpp
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

target_link_libraries(DextraControlboardUSB YARP::YARP_OS
                                            YARP::YARP_dev
                                            ROBOTICSLAB::ColorDebug
                                            YarpDevicesInterfaces)

yarp_install(TARGETS DextraControlboardUSB
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES DextraControlboardUSB.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
