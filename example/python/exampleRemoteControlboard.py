#! /usr/bin/env python

##
# @ingroup yarp_devices_examples_py
# \defgroup exampleRemoteControlboardPy exampleRemoteControlboard.py
#
# This example connects to a running \ref controlboardWrapper to move in Joint space.
#
# <b>Legal</b> 
#
# Copyright: (C) 2017 Universidad Carlos III de Madrid
#
# Author: Juan G Victores
#
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
#
# <b>Installation</b>
#
# We basically just need to give the module running (execution) permissions. In Ubuntu/Linux:
#
#\verbatim
#chmod +x $YARP_DEVICES_ROOT/example/python/exampleRemoteControlboard.py
#\endverbatim
#
# <b>Running</b> (assuming correct installation)
#
#\verbatim
#$YARP_DEVICES_ROOT/example/python/exampleRemoteControlboard.py
#\endverbatim
#
# <b>Modify</b>
#
# This file can be edited at $YARP_DEVICES_ROOT/example/python/exampleRemoteControlboard.py
#

import yarp  # imports YARP
yarp.Network.init()  # connect to YARP network
if yarp.Network.checkNetwork() != True:  # let's see if there was actually a reachable YARP network
    print '[error] Please try running yarp server'  # tell the user to start one with 'yarp server' if there isn't any
    quit()
options = yarp.Property()  # create an instance of Property, a nice YARP class for storing name-value (key-value) pairs
options.put('device','remote_controlboard')  # we add a name-value pair that indicates the YARP device
options.put('remote','/robotName')  # we add info on to whom we will connect
options.put('local','/exampleRemoteControlboard')  # we add info on how we will call ourselves on the YARP network
dd = yarp.PolyDriver(options)  # create a YARP multi-use driver with the given options

pos = dd.viewIPositionControl()  # make a position controller object we call 'pos'
vel = dd.viewIVelocityControl()  # make a velocity controller object we call 'vel'
enc = dd.viewIEncoders()  # make an encoder controller object we call 'enc'
mode = dd.viewIControlMode()  # make a operation mode controller object we call 'mode'

enc = dd.viewIEncoders()  # make an encoder controller object we call 'enc'
axes = enc.getAxes()  # retrieve number of joints

# use the object to set the device to position mode (as opposed to velocity mode)(note: stops the robot)
for j in range(axes):
    mode.setPositionMode(j)

print 'test positionMove(1,-35) -> moves motor 1 (start count at motor 0) to -35 degrees'
pos.positionMove(1,-35)

print 'test delay(5)'
yarp.Time.delay(5)

v = yarp.DVector(axes)  # create a YARP vector of doubles the size of the number of elements read by enc, call it 'v'
enc.getEncoders(v)  # read the encoder values and put them into 'v'
print 'v[1] is: ' + str(v[1])  # print element 1 of 'v', note that motors and encoders start at 0

# use the object to set the device to velocity mode (as opposed to position mode)
for j in range(1,axes):
    mode.setPositionMode(j)

print 'test velocityMove(0,10) -> moves motor 0 (start count at motor 0) at 10 degrees per second'
vel.velocityMove(0,10)

print 'test delay(5)'
yarp.Time.delay(5)

vel.velocityMove(0,0)  # stop the robot
dd.close()

yarp.Network.fini()  # disconnect from the YARP network
