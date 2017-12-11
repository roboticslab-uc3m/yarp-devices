# Copyright: 2014 UC3M
# Author: Raúl de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/checkCanBus.ini
                   conf/checkLocomotionCan0.ini
                   conf/checkLocomotionCan1.ini
                   conf/checkManipulationCan0.ini
                   conf/checkManipulationCan1.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR}/checkCanBus)
