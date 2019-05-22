#! /usr/bin/env python

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

############################################
# The following code needs https://github.com/robotology/yarp/commit/ca96cce20f8c04375a23090f4ec8fd0cb2a9d37d
# Not in 2.3.68, planned for 2.3.70
# As of Feb 21 2017, merged to YARP devel but not master.
############################################
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
