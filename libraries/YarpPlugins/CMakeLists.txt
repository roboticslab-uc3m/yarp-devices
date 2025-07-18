# FIXME: workaround for YARP 3.12.
if(YARP_VERSION VERSION_LESS 3.11.100)
    set(ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE OFF)
else()
    set(ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE ON)
endif()

option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Allow YARP to (re)build device param parsers"
       ${ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE})

# YARP devices.
add_subdirectory(AravisGigE)
add_subdirectory(CanBusBroker)
add_subdirectory(CanBusHico)
add_subdirectory(CanBusPeak)
add_subdirectory(CanBusSocket)
add_subdirectory(CuiAbsolute)
add_subdirectory(EmulatedControlBoard)
add_subdirectory(FakeJoint)
add_subdirectory(JointCalibrator)
add_subdirectory(Jr3Mbed)
add_subdirectory(Jr3Pci)
add_subdirectory(LacqueyFetch)
add_subdirectory(LeapMotionSensor)
add_subdirectory(PhidgetSpatial)
add_subdirectory(SpaceNavigator)
add_subdirectory(TechnosoftIpos)
add_subdirectory(TextilesHand)
add_subdirectory(WiimoteSensor)

# Port monitor plugins.
add_subdirectory(PortMonitorPlugins)
