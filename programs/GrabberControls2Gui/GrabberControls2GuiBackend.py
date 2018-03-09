"""
GrabberControls2GuiBackend
---------------------

Backend for the grabberControls2GUI

Author: David Estevez
Copyright: Universidad Carlos III de Madrid (C) 2017;
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""


import logging

import yarp


class GrabberControls2GuiBackend:
    def __init__(self, remote_port):
        self.remote_port = remote_port
        self.dd = None
        self.controls = None

    def init(self):
        # Check for YARP network
        yarp.Network.init()
        if not yarp.Network.checkNetwork():
            logging.error('Could not connect to YARP network. Please try running YARP server.')
            quit()

        # Create and configure driver
        options = yarp.Property()
        options.put('device','remote_grabber')
        options.put('remote', self.remote_port)
        options.put('local','/GrabberControls2Gui')
        self.dd = yarp.PolyDriver(options)

        # View driver as FrameGrabber
        self.controls = self.dd.viewIFrameGrabberControls2()

    def close(self):
        self.dd.close()
        yarp.Network.fini() # disconnect from the YARP network

    def set_zoom(self, zoom):
        print("Zoom set to {}".format(zoom))
        print(self.controls)
        self.controls.setFeature(yarp.YARP_FEATURE_ZOOM, zoom)

    def has_zoom(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_ZOOM)

    def set_focus(self, focus):
        print("Focus set to {}".format(focus))
        self.controls.setFeature(yarp.YARP_FEATURE_FOCUS, focus)

    def has_focus(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_FOCUS)

    def set_gain(self, gain):
        print("Gain set to {}".format(gain))
        self.controls.setFeature(yarp.YARP_FEATURE_GAIN, gain)

    def has_gain(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_GAIN)

    def set_exposure(self, exposure):
        print("Exposure set to {}".format(exposure))
        self.controls.setFeature(yarp.YARP_FEATURE_EXPOSURE, exposure)

    def has_exposure(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_EXPOSURE)

    def set_FPS(self, fps):
        print("FPS set to {}".format(fps))
        self.controls.setFeature(yarp.YARP_FEATURE_FRAME_RATE, fps)

    def has_FPS(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_FRAME_RATE)