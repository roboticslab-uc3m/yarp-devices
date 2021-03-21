# Copyright: (C) 2019 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Jennifer Joana Gago Muñoz
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(DextraSerialControlboard
                    CATEGORY device
                    TYPE roboticslab::DextraSerialControlboard
                    INCLUDE DextraSerialControlboard.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_DextraRawControlboardLib
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_DextraSerialControlboard)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(DextraSerialControlboard DextraSerialControlboard.hpp
                                             DeviceDriverImpl.cpp
                                             SerialSynapse.cpp)

    target_link_libraries(DextraSerialControlboard ROBOTICSLAB::DextraRawControlboardLib
                                                   YARP::YARP_os
                                                   YARP::YARP_dev
                                                   ROBOTICSLAB::ColorDebug)

    target_compile_features(DextraSerialControlboard PUBLIC cxx_std_11)

    yarp_install(TARGETS DextraSerialControlboard
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
