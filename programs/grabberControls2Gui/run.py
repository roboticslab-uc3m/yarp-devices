#!/usr/bin/env python3

"""
GrabberControls2Gui
---------------------

Simple GUI for controlling GigE cameras using Aravis and YARP

Author: David Estevez
Copyright: Universidad Carlos III de Madrid (C) 2017;
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""

import sys
import logging

import begin
import yarp
from PySide2 import QtWidgets

from .GrabberControls2GuiBackend import GrabberControls2GuiBackend
from .GrabberControls2GuiGUI import GrabberControls2GuiGUI


@begin.start(auto_convert=True)
@begin.logging
def main(remote_port: 'Remote port running the AravisGigE grabber'='/grabber'):
    # Check for YARP network
    yarp.Network.init()
    if not yarp.Network.checkNetwork():
        logging.error('Could not connect to YARP network. Please try running YARP server.')
        sys.exit(1)

    # Create and configure driver
    options = yarp.Property()
    options.put('device', 'remote_grabber')
    options.put('remote', remote_port)
    options.put('local', '/grabberControls2Gui')
    dd = yarp.PolyDriver(options)

    # View driver as FrameGrabber
    controls = dd.viewIFrameGrabberControls2()

    # Create Qt app
    app = QtWidgets.QApplication(sys.argv)

    # Create the widget and show it
    controller = GrabberControls2GuiBackend(controls)
    gui = GrabberControls2GuiGUI(controller)
    gui.show()

    # Run the app
    exit_code = app.exec_()

    dd.close()
    yarp.Network.fini()  # disconnect from the YARP network
    sys.exit(exit_code)
