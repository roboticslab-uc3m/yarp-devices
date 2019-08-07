# Copyright: (C) 2019 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Jennifer Joana Gago Muñoz
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraCanControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraCanControlboard
                    INCLUDE DextraCanControlboard.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_DextraRawControlboardLib)

if(NOT SKIP_DextraCanControlboard)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(DextraCanControlboard DextraCanControlboard.hpp
                                          DeviceDriverImpl.cpp
                                          CanSynapse.cpp)

    target_link_libraries(DextraCanControlboard DextraRawControlboardLib
                                                YARP::YARP_OS
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug
                                                YarpDevicesInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS DextraCanControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS DextraCanControlboard
                     LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES DextraCanControlboard.ini
                     DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

endif()
