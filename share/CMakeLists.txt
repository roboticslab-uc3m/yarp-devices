# Copyright: 2019 UC3M
# Author: Juan G Victores and Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0

set(applications)
set(contexts)

if(ENABLE_dumpCanBus)
    list(APPEND contexts contexts/dumpCanBus)
endif()

if(ENABLE_launchCanBus)
    list(APPEND contexts contexts/launchCanBus)
endif()

if(ENABLE_SpaceNavigator)
    list(APPEND applications applications/yarpmanager/spaceNavigator)
endif()

yarp_install(FILES applications/yarpmanager/ymanager.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR}/yarpmanager)

yarp_install(DIRECTORY ${applications}
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR}/yarpmanager)

yarp_install(DIRECTORY ${contexts}
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR})
