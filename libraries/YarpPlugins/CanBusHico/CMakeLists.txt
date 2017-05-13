# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CanBusHico
                    CATEGORY device
                    TYPE roboticslab::CanBusHico
                    INCLUDE CanBusHico.hpp
                    DEFAULT ON)

if(NOT SKIP_CanBusHico)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs})

yarp_add_plugin(CanBusHico CanBusHico.cpp
                           CanBusHico.hpp
                           hico_api.h
                           DeviceDriverImpl.cpp
                           HicoCanMessage.cpp
                           HicoCanMessage.hpp)

add_dependencies(CanBusHico COLOR_DEBUG)

target_link_libraries(CanBusHico YARP::YARP_OS
                                 YARP::YARP_dev)

yarp_install(TARGETS CanBusHico
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CanBusHico.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
