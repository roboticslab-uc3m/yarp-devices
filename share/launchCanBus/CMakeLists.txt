# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/oneCanBusOneWrapper.ini
                   conf/oneCanBusOneWrapper-CuiAbsolute.ini
                   conf/oneCanBusOneWrapper-left-arm.ini
                   conf/oneCanBusOneWrapper-right-arm.ini
                   conf/twoCanBusThreeWrappers.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR}/launchCanBus)
