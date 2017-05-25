# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(roboticslab-yarp-devices)

### Go through single applications

if(ENABLE_dumpCanBus)
    add_subdirectory(dumpCanBus)
endif()

if(ENABLE_checkCanBus)
    add_subdirectory(checkCanBus)
endif()

if(ENABLE_oneCanBusOneWrapper)
    add_subdirectory(oneCanBusOneWrapper)
endif()

if(ENABLE_launchManipulation)
    add_subdirectory(launchManipulation)
endif()

if(ENABLE_launchLocomotion)
    add_subdirectory(launchLocomotion)
endif()

if(ENABLE_tests)
    add_subdirectory(testCuiAbsolute)
endif()

if(ENABLE_SpaceNavigator)
    add_subdirectory(spaceNavigator)
endif()

