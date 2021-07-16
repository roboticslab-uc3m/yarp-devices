# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico

yarp_prepare_plugin(EmulatedControlboard
                    CATEGORY device
                    TYPE roboticslab::EmulatedControlboard
                    INCLUDE EmulatedControlboard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_EmulatedControlboard)

    yarp_add_plugin(EmulatedControlboard EmulatedControlboard.hpp
                                         DeviceDriverImpl.cpp
                                         IControlLimitsImpl.cpp
                                         IControlModeImpl.cpp
                                         IEncodersImpl.cpp
                                         IPositionControlImpl.cpp
                                         IPositionDirectImpl.cpp
                                         IVelocityControlImpl.cpp
                                         PeriodicThreadImpl.cpp
                                         SharedArea.cpp)

    target_link_libraries(EmulatedControlboard YARP::YARP_os
                                               YARP::YARP_dev)

    yarp_install(TARGETS EmulatedControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
