#! /usr/bin/env python

#Check for YARP network
yarp.Network.init()
if not yarp.Network.checkNetwork():
    print('Could not connect to YARP network. Please try running YARP server.')
    quit()

# Create and configure driver
options = yarp.Property()
options.put('device','remote_grabber')
options.put('remote', '/grabber')
options.put('local','/GrabberControls2Gui')
dd = yarp.PolyDriver(options)

# View driver as FrameGrabber
controls = dd.viewIFrameGrabberControls2()

# Check if a feature exists
print(controls.hasFeature(yarp.YARP_FEATURE_GAIN))

# Get the current value of a feature
gain = controls.getFeature(yarp.YARP_FEATURE_GAIN)
print(gain)

# Set a new value for a feature
controls.setFeature(yarp.YARP_FEATURE_GAIN, 10)

# Close device and disconnect from the YARP network
dd.close()
yarp.Network.fini()
