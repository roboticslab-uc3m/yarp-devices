# Copyright: (C) 2019 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Jennifer Joana Gago Muñoz
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraCanControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraCanControlboard
                    INCLUDE DextraCanControlboard.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_CanBusSharerLib;ENABLE_DextraRawControlboardLib")

if(NOT SKIP_DextraCanControlboard)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(DextraCanControlboard DextraCanControlboard.hpp
                                          DeviceDriverImpl.cpp
                                          ICanBusSharerImpl.cpp
                                          CanSynapse.cpp)

    target_link_libraries(DextraCanControlboard DextraRawControlboardLib
                                                YARP::YARP_os
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug
                                                CanBusSharerLib)

    target_compile_features(DextraCanControlboard PRIVATE cxx_std_11)

    yarp_install(TARGETS DextraCanControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
