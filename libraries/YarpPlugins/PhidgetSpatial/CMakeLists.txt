# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(Phidgets 2.1 EXACT QUIET)

if(NOT Phidgets_FOUND AND (NOT DEFINED ENABLE_PhidgetSpatial OR PhidgetSpatial))
    message(WARNING "Phidgets package not found, disabling PhidgetSpatial")
endif()

yarp_prepare_plugin(PhidgetSpatial
                    CATEGORY device
                    TYPE roboticslab::PhidgetSpatial
                    INCLUDE PhidgetSpatial.hpp
                    DEFAULT ON
                    DEPENDS "NOT CMAKE_VERSION VERSION_LESS 2.8.11;Phidgets_FOUND")

if(NOT SKIP_PhidgetSpatial)

    # Include any directories needed for YARP
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(PhidgetSpatial PhidgetSpatial.hpp
                                   PhidgetSpatial.cpp
                                   DeviceDriverImpl.cpp
                                   IAnalogSensorImpl.cpp)

    target_link_libraries(PhidgetSpatial YARP::YARP_OS
                                         YARP::YARP_dev
                                         Phidgets::Phidget21)

    yarp_install(TARGETS PhidgetSpatial
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES PhidgetSpatial.ini
                 COMPONENT runtime
                 DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_PhidgetSpatial OFF CACHE BOOL "Enable/disable PhidgetSpatial device" FORCE)

endif()