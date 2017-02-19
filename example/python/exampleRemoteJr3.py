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
    sys.exit()

iAnalogSensor = dd.viewIAnalogSensor()

print 'delay(1)'
yarp.Time.delay(1)

print 'bye!'


