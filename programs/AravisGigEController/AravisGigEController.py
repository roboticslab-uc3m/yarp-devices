#!/usr/bin/env python3

"""
AravisGigEController
---------------------

Simple GUI for controlling GigE cameras using Aravis and YARP

Author: David Estevez
Copyright: Universidad Carlos III de Madrid (C) 2017;
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""

import logging

import yarp
import begin


@begin.start(auto_convert=True, config_file='config.txt')
@begin.logging
def main(remote_port: 'Remote port running the AravisGigE grabber'='/grabber'):

    # Check for YARP network
    yarp.Network.init()
    if not yarp.Network.checkNetwork():
        logging.error('Could not connect to YARP network. Please try running YARP server.')
        quit()

    # Create and configure driver
    options = yarp.Property()
    options.put('device','remote_controlboard')
    options.put('remote', remote_port)
    options.put('local','/AravisGigEController')
    dd = yarp.PolyDriver(options)

