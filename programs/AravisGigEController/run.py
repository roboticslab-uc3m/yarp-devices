#!/usr/bin/env python3

"""
AravisGigEController
---------------------

Simple GUI for controlling GigE cameras using Aravis and YARP

Author: David Estevez
Copyright: Universidad Carlos III de Madrid (C) 2017;
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""

import sys

import begin
from PySide import QtGui

from .AravisGigEController import AravisGigEController
from .AravisGigEControllerGUI import AravisGigEControllerGUI


@begin.start(auto_convert=True)
@begin.logging
def main(remote_port: 'Remote port running the AravisGigE grabber'='/grabber'):
    # Create Qt app
    app = QtGui.QApplication(sys.argv)

    # Create the widget and show it
    controller = AravisGigEController(remote_port)
    gui = AravisGigEControllerGUI(controller)
    gui.show()

    # Run the app
    sys.exit(app.exec_())