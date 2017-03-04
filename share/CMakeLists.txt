# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(yarp-devices)

### Go through single applications

add_subdirectory(dumpCanBus)
add_subdirectory(checkCanBus)
add_subdirectory(testCuiAbsolute)

add_subdirectory(oneCanBusOneWrapper)

add_subdirectory(launchManipulation)
add_subdirectory(launchLocomotion)

add_subdirectory(playbackManipulation)
add_subdirectory(recordManipulation)

add_subdirectory(playbackLocomotion)
add_subdirectory(recordLocomotion)

add_subdirectory(Playback)

