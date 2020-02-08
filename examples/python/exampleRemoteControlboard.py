#! /usr/bin/env python

##
# @ingroup yarp_devices_examples_py
# @defgroup exampleRemoteControlboardPy exampleRemoteControlboard.py
#
# @brief This example connects to a remote @ref controlboard to move in joint space.
#
# <b>Requires YARP 3.0.</b>
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
#yarpdev --device YarpOpenraveControlboard --env data/lab1.env.xml --robotIndex 0 --manipulatorIndex 0 --name /BarrettWAM/arm --view
#$YARP_DEVICES_ROOT/example/python/exampleRemoteControlboard.py
#\endverbatim
#
# <b>Modify</b>
#
# This file can be edited at $YARP_DEVICES_ROOT/example/python/exampleRemoteControlboard.py
#

import yarp  # imports YARP
yarp.Network.init()  # connect to YARP network
if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
    print '[error] Please try running yarp server'  # tell the user to start one with 'yarp server' if there isn't any
    quit()
options = yarp.Property()  # create an instance of Property, a nice YARP class for storing name-value (key-value) pairs
options.put('device','remote_controlboard')  # we add a name-value pair that indicates the YARP device
options.put('remote','/BarrettWAM/arm')  # we add info on to whom we will connect
options.put('local','/exampleRemoteControlboard')  # we add info on how we will call ourselves on the YARP network
dd = yarp.PolyDriver(options)  # create a YARP multi-use driver with the given options
if not dd.isValid():
    print '[error] Please launch robot side'
    quit()

pos = dd.viewIPositionControl()  # make a position controller object we call 'pos'
vel = dd.viewIVelocityControl()  # make a velocity controller object we call 'vel'
posd = dd.viewIPositionDirect()  # make a direct position controller object we call 'posd'
enc = dd.viewIEncoders()  # make an encoder controller object we call 'enc'
mode = dd.viewIControlMode()  # make a operation mode controller object we call 'mode'
ll = dd.viewIControlLimits()  # make a limits controller object we call 'll'

axes = enc.getAxes()  # retrieve number of joints

lmin = yarp.DVector(1)
lmax = yarp.DVector(1)
ll.getLimits(0,lmin,lmax) # retrieve limits of joint 0
print('lmin',lmin[0],'lmax',lmax[0])

# use the object to set the device to position mode (as opposed to velocity mode)(note: stops the robot)
mode.setControlModes(yarp.IVector(axes, yarp.encode('pos')))

print 'positionMove(1,-35) -> moves motor 1 (start count at motor 0) to -35 degrees'
pos.positionMove(1,-35)

done = False
while not done:
    print 'wait to reach...'
    yarp.delay(1.0) # [s]
    done = pos.checkMotionDone()

v = yarp.DVector(axes)  # create a YARP vector of doubles the size of the number of elements read by enc, call it 'v'
enc.getEncoders(v)  # read the encoder values and put them into 'v'
print 'v[1] is: ' + str(v[1])  # print element 1 of 'v', note that motors and encoders start at 0

targets = list(range(0,10+5*axes,5))
print 'positionMove(...) -> [multiple axes] moves motor 0 to 10 degrees, motor 1 to 15 degrees and so on'
pos.positionMove(yarp.DVector(targets))

done = False
while not done:
    print 'wait to reach...'
    yarp.delay(1.0) # [s]
    done = pos.checkMotionDone()

# use the object to set the device to velocity mode (as opposed to position mode)
mode.setControlModes(yarp.IVector(axes, yarp.encode('vel')))

print 'velocityMove(0,10) -> moves motor 0 (start count at motor 0) at 10 degrees per second'
vel.velocityMove(0,10)

print 'delay(5)'
yarp.delay(5)

vel.velocityMove(0,0)  # stop the robot

mode.setControlModes(yarp.IVector(axes, yarp.encode('posd')))
home = [0] * axes
print 'positionMove(home) -> [multiple axes] moves all to 0 immediately'
posd.setPositions(yarp.DVector(home))

dd.close()

yarp.Network.fini()  # disconnect from the YARP network
