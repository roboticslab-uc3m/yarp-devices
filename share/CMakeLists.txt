# Copyright: 2019 UC3M
# Author: Juan G Victores and Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0

if(ENABLE_SpaceNavigator)
    yarp_install(FILES applications/spaceNavigator.xml
                 DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
endif()

yarp_install(FILES applications/ymanager.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
