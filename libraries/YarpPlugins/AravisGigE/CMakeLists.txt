# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & David Estevez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(ARAVIS QUIET)

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

    get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                        ${_common_dirs}
                        ${ARAVIS_INCLUDE_DIRS})

    yarp_add_plugin(AravisGigE AravisGigE.hpp
                               DeviceDriverImpl.cpp
                               IFrameGrabberControls2Impl.cpp
                               IFrameGrabberImageRawImpl.cpp
                               IFrameGrabberImpl.cpp)

    add_dependencies(AravisGigE COLOR_DEBUG)

    target_link_libraries(AravisGigE YARP::YARP_OS
                                     YARP::YARP_dev
                                     ${ARAVIS_LIBRARIES})

    yarp_install(TARGETS AravisGigE
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES AravisGigE.ini
                 COMPONENT runtime
                 DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AravisGigE OFF CACHE BOOL "Enable/disable AravisGigE device" FORCE)

endif()
