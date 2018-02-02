# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(PhidgetSpatial
                    CATEGORY device
                    TYPE roboticslab::PhidgetSpatial
                    INCLUDE PhidgetSpatial.hpp
                    DEFAULT OFF)

IF (NOT SKIP_PhidgetSpatial)

    # Include any directories needed for YARP
    include_directories( ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

    yarp_add_plugin(PhidgetSpatial PhidgetSpatial.hpp PhidgetSpatial.cpp DeviceDriverImpl.cpp IAnalogSensorImpl.cpp )

    TARGET_LINK_LIBRARIES(PhidgetSpatial ${YARP_LIBRARIES} phidget21)

    yarp_install(TARGETS PhidgetSpatial
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES PhidgetSpatial.ini
                 COMPONENT runtime
                 DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_PhidgetSpatial)
