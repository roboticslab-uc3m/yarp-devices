# Copyright: 2019 UC3M
# Author: Juan G Victores and Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0

if(ENABLE_dumpCanBus)
    yarp_install(DIRECTORY contexts/dumpCanBus
                 DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR})
endif()

if(ENABLE_launchCanBus)
    yarp_install(DIRECTORY contexts/launchCanBus
                 DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR})
endif()

if(ENABLE_SpaceNavigator)
    yarp_install(DIRECTORY applications/yarpmanager/spaceNavigator
                 DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR}/yarpmanager)
endif()

yarp_install(FILES applications/yarpmanager/ymanager.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR}/yarpmanager)
