# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico

yarp_prepare_plugin(FakeControlboard
                    CATEGORY device
                    TYPE roboticslab::FakeControlboard
                    INCLUDE FakeControlboard.hpp
                    DEFAULT ON)

if(NOT SKIP_FakeControlboard)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(FakeControlboard FakeControlboard.hpp
                                 FakeControlboard.cpp
                                 DeviceDriverImpl.cpp
                                 IControlLimitsImpl.cpp
                                 IControlModeImpl.cpp
                                 IEncodersImpl.cpp
                                 IEncodersTimedImpl.cpp
                                 IPositionControlImpl.cpp
                                 IPositionDirectImpl.cpp
                                 IRemoteVariablesImpl.cpp
                                 ITorqueControlImpl.cpp
                                 IVelocityControlImpl.cpp
                                 PeriodicThreadImpl.cpp
                                 SharedArea.cpp)

target_link_libraries(FakeControlboard YARP::YARP_OS
                                       YARP::YARP_dev
                                       ROBOTICSLAB::ColorDebug
                                       YarpDevicesInterfaces)

yarp_install(TARGETS FakeControlboard
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES FakeControlboard.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
