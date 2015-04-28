# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(teo-body)

### Go through single applications

add_subdirectory(exampleCanBusControlboard)

add_subdirectory(manipulation)
add_subdirectory(launchManipulation)
add_subdirectory(playbackManipulation)
add_subdirectory(recordManipulation)

add_subdirectory(locomotion)
add_subdirectory(launchLocomotion)
add_subdirectory(playbackLocomotion)
add_subdirectory(recordLocomotion)

add_subdirectory(playbackServer)

