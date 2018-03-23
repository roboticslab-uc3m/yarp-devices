# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# Find YARP (common dependency for all devices).
find_package(YARP REQUIRED)

if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
    include(YarpPlugin)
    include_directories(${YARP_INCLUDE_DIRS})
endif()

# Create device interface target.
add_library(YarpDevicesInterfaces INTERFACE)

# Configure usage requirements.
target_include_directories(YarpDevicesInterfaces INTERFACE ${YARP_INCLUDE_DIRS}
                                                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install interface headers.
install(FILES ICanBusHico.h
              ICanBusSharer.h
              ICuiAbsolute.h
              IProximitySensors.h
              ITechnosoftIpos.h
              hico_api.h # remove, see #122
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Register export set.
install(TARGETS YarpDevicesInterfaces
        EXPORT ROBOTICSLAB_YARP_DEVICES)

# Export variables to the global scope.
set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_YARP_DEVICES_TARGETS YarpDevicesInterfaces)

# YARP devices.
add_subdirectory(AravisGigE)
add_subdirectory(CanBusControlboard)
add_subdirectory(CanBusHico)
add_subdirectory(CuiAbsolute)
add_subdirectory(FakeControlboard)
add_subdirectory(FakeJoint)
add_subdirectory(Jr3)
add_subdirectory(LacqueyFetch)
add_subdirectory(LeapMotionSensor)
add_subdirectory(PhidgetSpatial)
add_subdirectory(ProximitySensorsClient)
add_subdirectory(SpaceNavigator)
add_subdirectory(TechnosoftIpos)
add_subdirectory(TextilesHand)
add_subdirectory(WiimoteSensor)

# Port monitor plugins.
add_subdirectory(PortMonitorPlugins)
