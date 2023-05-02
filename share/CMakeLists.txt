if(ENABLE_SpaceNavigator)
    yarp_install(FILES applications/spaceNavigator.xml
                 DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
endif()

yarp_install(FILES applications/ymanager.ini
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
