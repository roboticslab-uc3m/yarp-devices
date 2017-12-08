# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/launchManipulation.ini
                   conf/launchManipulation-fake.ini
                   conf/launchManipulation_only_head.ini
                   conf/launchManipulation_wo_cuis.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR}/launchManipulation)
