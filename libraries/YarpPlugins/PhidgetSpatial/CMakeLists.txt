# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT Phidgets_FOUND AND (NOT DEFINED ENABLE_PhidgetSpatial OR PhidgetSpatial))
    message(WARNING "Phidgets package not found, disabling PhidgetSpatial")
endif()

yarp_prepare_plugin(PhidgetSpatial
                    CATEGORY device
                    TYPE roboticslab::PhidgetSpatial
                    INCLUDE PhidgetSpatial.hpp
                    DEFAULT ON
                    DEPENDS Phidgets_FOUND
                    EXTRA_CONFIG WRAPPER=analogServer)

if(NOT SKIP_PhidgetSpatial)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(PhidgetSpatial PhidgetSpatial.hpp
                                   PhidgetSpatial.cpp
                                   DeviceDriverImpl.cpp
                                   IAnalogSensorImpl.cpp)

    target_link_libraries(PhidgetSpatial YARP::YARP_OS
                                         YARP::YARP_dev
                                         Phidgets::Phidget21)

    yarp_install(TARGETS PhidgetSpatial
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_PhidgetSpatial OFF CACHE BOOL "Enable/disable PhidgetSpatial device" FORCE)

endif()
