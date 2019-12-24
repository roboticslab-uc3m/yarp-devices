# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico

yarp_prepare_plugin(FakeControlboard
                    CATEGORY device
                    TYPE roboticslab::FakeControlboard
                    INCLUDE FakeControlboard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_FakeControlboard)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

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
                                           ROBOTICSLAB::ColorDebug)

    yarp_install(TARGETS FakeControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
