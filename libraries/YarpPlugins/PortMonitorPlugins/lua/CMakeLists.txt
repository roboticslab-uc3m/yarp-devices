if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    include(YarpInstallationHelpers)
endif()

yarp_configure_external_installation(roboticslab-yarp-devices)

yarp_install(FILES amor_sensors_modifier.lua
             DESTINATION ${ROBOTICSLAB-YARP-DEVICES_CONTEXTS_INSTALL_DIR}/portmonitor)
