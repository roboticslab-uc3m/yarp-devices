# FIXME: workaround for YARP 3.12+.
option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Allow YARP to (re)build device param parsers" ON)

# YARP devices.
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
# deprecated, see https://github.com/roboticslab-uc3m/yarp-devices/issues/265
# add_subdirectory(LeapMotionSensor)
add_subdirectory(PhidgetSpatial)
add_subdirectory(SpaceNavigator)
add_subdirectory(TechnosoftIpos)
add_subdirectory(TextilesHand)
add_subdirectory(Wiimote)

# Port monitor plugins.
add_subdirectory(PortMonitorPlugins)
