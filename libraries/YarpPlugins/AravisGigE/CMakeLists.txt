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
                    DEPENDS ARAVIS_FOUND)

if(NOT SKIP_AravisGigE)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                        ${ARAVIS_INCLUDE_DIRS})

    link_directories(${ARAVIS_LIBRARY_DIRS})

    yarp_add_plugin(AravisGigE AravisGigE.hpp
                               DeviceDriverImpl.cpp
                               IFrameGrabberControls2Impl.cpp
                               IFrameGrabberImageRawImpl.cpp
                               IFrameGrabberImpl.cpp)

    target_link_libraries(AravisGigE YARP::YARP_OS
                                     YARP::YARP_dev
                                     ${ARAVIS_LIBRARIES}
                                     ROBOTICSLAB::ColorDebug)

    yarp_install(TARGETS AravisGigE
                 COMPONENT runtime
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES AravisGigE.ini
                 COMPONENT runtime
                 DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AravisGigE OFF CACHE BOOL "Enable/disable AravisGigE device" FORCE)

endif()
