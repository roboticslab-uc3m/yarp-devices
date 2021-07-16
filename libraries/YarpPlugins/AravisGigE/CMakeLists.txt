# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & David Estevez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT ARAVIS_FOUND AND (NOT DEFINED ENABLE_AravisGigE OR ENABLE_AravisGigE))
    message(WARNING "ARAVIS package not found, disabling AravisGigE")
endif()

yarp_prepare_plugin(AravisGigE
                    CATEGORY device
                    TYPE roboticslab::AravisGigE
                    INCLUDE AravisGigE.hpp
                    DEFAULT ON
                    DEPENDS ARAVIS_FOUND
                    EXTRA_CONFIG WRAPPER=grabberDual)

if(NOT SKIP_AravisGigE)

    link_directories(${ARAVIS_LIBRARY_DIRS})

    yarp_add_plugin(AravisGigE AravisGigE.hpp
                               DeviceDriverImpl.cpp
                               IFrameGrabberControlsImpl.cpp
                               IFrameGrabberImageRawImpl.cpp)

    target_link_libraries(AravisGigE YARP::YARP_os
                                     YARP::YARP_dev
                                     ${ARAVIS_LIBRARIES})

    target_include_directories(AravisGigE PRIVATE ${ARAVIS_INCLUDE_DIRS})

    yarp_install(TARGETS AravisGigE
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AravisGigE OFF CACHE BOOL "Enable/disable AravisGigE device" FORCE)

endif()
