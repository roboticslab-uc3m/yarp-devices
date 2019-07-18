# Copyright: (C) 2019 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Jennifer Joana Gago Muñoz
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraSerialControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraSerialControlboard
                    INCLUDE DextraSerialControlboard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_DextraSerialControlboard)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(DextraSerialControlboard DextraSerialControlboard.cpp
                                             DextraSerialControlboard.hpp
                                             DeviceDriverImpl.cpp
                                             IAxisInfoImpl.cpp
                                             IControlLimitsImpl.cpp
                                             IControlModeImpl.cpp
                                             IEncodersTimedImpl.cpp
                                             IPositionControlImpl.cpp
                                             IPositionDirectImpl.cpp
                                             Synapse.hpp
                                             Synapse.cpp)

    target_link_libraries(DextraSerialControlboard YARP::YARP_OS
                                                   YARP::YARP_dev
                                                   ROBOTICSLAB::ColorDebug)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS DextraSerialControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS DextraSerialControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES DextraSerialControlboard.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
