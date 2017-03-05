# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname Playback)

file(GLOB txt ${CMAKE_CURRENT_SOURCE_DIR}/txt/*.txt)

yarp_install(FILES ${txt} DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR}/${appname}/txt)

