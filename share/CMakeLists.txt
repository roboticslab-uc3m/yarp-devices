# Copyright: 2017 UC3M
# Author: Juan G Victores and Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES ymanager.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DATA_INSTALL_DIR})

### Go through single applications

if(ENABLE_checkCanBus)
    add_subdirectory(checkCanBus)
endif()

if(ENABLE_dumpCanBus)
    add_subdirectory(dumpCanBus)
endif()

if(ENABLE_launchCanBus)
    add_subdirectory(launchCanBus)
endif()

if(ENABLE_SpaceNavigator)
    add_subdirectory(spaceNavigator)
endif()

if(ENABLE_tests)
    add_subdirectory(testCuiAbsolute)
endif()
