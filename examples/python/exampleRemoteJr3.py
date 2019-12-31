#! /usr/bin/env python

##
# @ingroup yarp_devices_examples_py
# @defgroup exampleRemoteJr3Py exampleRemoteJr3.py
# @brief This example connects to a remote @ref Jr3 device.

import yarp

yarp.Network.init()

if yarp.Network.checkNetwork() != True:
    print '[error] Please try running yarp server'
    quit()

options = yarp.Property()
options.put('device','analogsensorclient')
options.put('remote','/jr3/ch0:o')
options.put('local','/jr3/ch0:i')
dd = yarp.PolyDriver(options)  # calls open -> connects

if not dd.isValid():
    print 'Cannot open the device!'
    quit()

iAnalogSensor = dd.viewIAnalogSensor()

# The following delay should avoid 0 channels and bad read
print 'delay(1)'
yarp.Time.delay(1)

channels = iAnalogSensor.getChannels()
print 'channels:', channels

# Of course we dislike while(1)
while 1:
    vector = yarp.Vector(channels)
    iAnalogSensor.read(vector)
    vector_list = []
    for i in range(vector.size()):
        vector_list.append( vector[i] )
    print '[%s]' % ', '.join(map(str, vector_list))

print 'bye!'
