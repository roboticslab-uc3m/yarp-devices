# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusFake
                    CATEGORY device
                    TYPE roboticslab::CanBusFake
                    INCLUDE CanBusFake.hpp
                    DEFAULT ON)

if(NOT SKIP_CanBusFake)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(CanBusFake CanBusFake.hpp
                               CanBusFake.cpp
                               FakeCanMessage.hpp
                               FakeCanMessage.cpp)

    target_link_libraries(CanBusFake YARP::YARP_dev)

    yarp_install(TARGETS CanBusFake
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
