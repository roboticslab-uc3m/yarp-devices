#! /usr/bin/env python

##
# @ingroup yarp_devices_examples_py
# @defgroup exampleRemoteGrabberPy exampleRemoteGrabber.py
# @brief This example connects to a remote grabber device.

import yarp  # imports YARP
yarp.Network.init() #Check for YARP network
if not yarp.Network.checkNetwork():
    print('Could not connect to YARP network. Please try running YARP server.')
    quit()

# Create and configure driver
options = yarp.Property()
options.put('device','remote_grabber')
options.put('remote', '/grabber')
options.put('local','/python/grabber')
dd = yarp.PolyDriver(options)
if not dd.isValid():
    print '[error] Please launch camera side'
    quit()

# View grabber interfaces
grabberControls = dd.viewIFrameGrabberControls()
grabber = dd.viewIFrameGrabberImage()

# Check if a feature exists
print(grabberControls.hasFeature(yarp.YARP_FEATURE_GAIN))

# Get the current value of a feature
gain = grabberControls.getFeature(yarp.YARP_FEATURE_GAIN)
print(gain)

# Set a new value for a feature
grabberControls.setFeature(yarp.YARP_FEATURE_GAIN, 10)

# Obtain an image
yarp.delay(0.5) # May have to previously wait
yarpImg = yarp.ImageRgb()
grabber.getImage(yarpImg)

# Close device and disconnect from the YARP network
dd.close()
yarp.Network.fini()
