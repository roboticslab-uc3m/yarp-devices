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
    def __init__(self, controls):
        self.controls = controls

    def init(self):
        pass

    def close(self):
        pass

    def set_zoom(self, zoom):
        logging.debug("Zoom set to {}".format(zoom))
        logging.debug(self.controls)
        self.controls.setFeature(yarp.YARP_FEATURE_ZOOM, zoom)

    def has_zoom(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_ZOOM)

    def set_focus(self, focus):
        logging.debug("Focus set to {}".format(focus))
        self.controls.setFeature(yarp.YARP_FEATURE_FOCUS, focus)

    def has_focus(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_FOCUS)

    def set_gain(self, gain):
        logging.debug("Gain set to {}".format(gain))
        self.controls.setFeature(yarp.YARP_FEATURE_GAIN, gain)

    def has_gain(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_GAIN)

    def set_exposure(self, exposure):
        logging.debug("Exposure set to {}".format(exposure))
        self.controls.setFeature(yarp.YARP_FEATURE_EXPOSURE, exposure)

    def has_exposure(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_EXPOSURE)

    def set_FPS(self, fps):
        logging.debug("FPS set to {}".format(fps))
        self.controls.setFeature(yarp.YARP_FEATURE_FRAME_RATE, fps)

    def has_FPS(self):
        return self.controls.hasFeature(yarp.YARP_FEATURE_FRAME_RATE)
