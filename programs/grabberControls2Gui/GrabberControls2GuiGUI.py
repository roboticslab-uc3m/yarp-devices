"""
AravisGigEController
---------------------

Frontend for the AravisGigEController

Author: David Estevez
Copyright: Universidad Carlos III de Madrid (C) 2017;
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""

import os, sys

from PySide2 import QtCore, QtGui, QtWidgets
from PySide2 import QtUiTools


def load_ui(file_name, where=None):
    """
    Loads a .UI file into the corresponding Qt Python object
    :param file_name: UI file path
    :param where: Use this parameter to load the UI into an existing class (i.e. to override methods)
    :return: loaded UI
    """
    # Create a QtLoader
    loader = QtUiTools.QUiLoader()

    # Open the UI file
    ui_file = QtCore.QFile(file_name)
    ui_file.open(QtCore.QFile.ReadOnly)

    # Load the contents of the file
    ui = loader.load(ui_file, where)

    # Close the file
    ui_file.close()

    return ui


class GrabberControls2GuiGUI(QtWidgets.QWidget):
    def __init__(self, controller=None, parent=None):
        QtWidgets.QWidget.__init__(self, parent)

        self.zoomSlider = None
        self.zoomSpinBox = None
        self.focusSlider = None
        self.focusSpinBox = None
        self.gainSlider = None
        self.gainSpinBox = None
        self.exposureSlider = None
        self.exposureSpinBox = None
        self.fpsSlider = None
        self.fpsSpinBox = None

        self.controller = controller
        self.controller.init()

        self.setupUI()
        self.resetValues()

    def setupUI(self):
        # Load UI and set it as main layout
        ui_file_path = os.path.join(os.path.realpath(os.path.dirname(__file__)), 'templates', 'GrabberControls2GuiGUI.ui')
        main_widget = load_ui(ui_file_path, self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(main_widget)
        self.setLayout(layout)

        # Get a reference to all required widgets
        self.zoomSlider = self.findChild(QtWidgets.QSlider, 'zoomSlider')
        self.zoomSpinBox = self.findChild(QtWidgets.QDoubleSpinBox, 'zoomSpinBox')
        self.focusSlider = self.findChild(QtWidgets.QSlider, 'focusSlider')
        self.focusSpinBox = self.findChild(QtWidgets.QDoubleSpinBox, 'focusSpinBox')
        self.gainSlider = self.findChild(QtWidgets.QSlider, 'gainSlider')
        self.gainSpinBox = self.findChild(QtWidgets.QDoubleSpinBox, 'gainSpinBox')
        self.exposureSlider = self.findChild(QtWidgets.QSlider, 'exposureSlider')
        self.exposureSpinBox = self.findChild(QtWidgets.QDoubleSpinBox, 'exposureSpinBox')
        self.fpsSlider = self.findChild(QtWidgets.QSlider, 'fpsSlider')
        self.fpsSpinBox = self.findChild(QtWidgets.QDoubleSpinBox, 'fpsSpinBox')

        # Configure widget ranges:
        max_float = sys.float_info.max
        self.zoomSlider.setMinimum(0)
        self.zoomSlider.setMaximum(100)
        self.zoomSpinBox.setMinimum(0)
        self.zoomSpinBox.setMaximum(100)
        self.focusSlider.setMinimum(0)
        self.focusSlider.setMaximum(1000)
        self.focusSpinBox.setMinimum(0)
        self.focusSpinBox.setMaximum(1000)
        self.gainSlider.setMinimum(0)
        self.gainSlider.setMaximum(30) # Not sure about this one, have to check with actual hardware
        self.gainSpinBox.setMinimum(0)
        self.gainSpinBox.setMaximum(30)
        self.exposureSlider.setMinimum(0)
        self.exposureSlider.setMaximum(100000) # Not sure again
        self.exposureSpinBox.setMinimum(0)
        self.exposureSpinBox.setMaximum(100000)
        self.fpsSlider.setMinimum(0)
        self.fpsSlider.setMaximum(30) # Not sure
        self.fpsSpinBox.setMinimum(0)
        self.fpsSpinBox.setMaximum(30)

        # Disable widgets if feature not supported
        self.zoomSlider.setEnabled(self.controller.has_zoom())
        self.zoomSpinBox.setEnabled(self.controller.has_zoom())
        self.focusSlider.setEnabled(self.controller.has_focus())
        self.focusSpinBox.setEnabled(self.controller.has_focus())
        self.gainSlider.setEnabled(self.controller.has_gain())
        self.gainSpinBox.setEnabled(self.controller.has_gain())
        self.exposureSlider.setEnabled(self.controller.has_exposure())
        self.exposureSpinBox.setEnabled(self.controller.has_exposure())
        self.fpsSlider.setEnabled(self.controller.has_FPS())
        self.fpsSpinBox.setEnabled(self.controller.has_FPS())

        # Connect to signals:
        self.zoomSlider.valueChanged.connect(self.onZoomSliderChanged)
        self.zoomSpinBox.valueChanged.connect(self.onZoomSpinBoxChanged)
        self.focusSlider.valueChanged.connect(self.onFocusSliderChanged)
        self.focusSpinBox.valueChanged.connect(self.onFocusSpinBoxChanged)
        self.gainSlider.valueChanged.connect(self.onGainSliderChanged)
        self.gainSpinBox.valueChanged.connect(self.onGainSpinBoxChanged)
        self.exposureSlider.valueChanged.connect(self.onExposureSliderChanged)
        self.exposureSpinBox.valueChanged.connect(self.onExposureSpinBoxChanged)
        self.fpsSlider.valueChanged.connect(self.onfpsSliderChanged)
        self.fpsSpinBox.valueChanged.connect(self.onfpsSpinBoxChanged)

    def resetValues(self):
        self.zoomSlider.setValue(0)
        self.zoomSpinBox.setValue(0)
        self.focusSlider.setValue(0)
        self.focusSpinBox.setValue(0)
        self.gainSlider.setValue(0)
        self.gainSpinBox.setValue(0)
        self.exposureSlider.setValue(0)
        self.exposureSpinBox.setValue(0)
        self.fpsSlider.setValue(0)
        self.fpsSpinBox.setValue(0)

    def onZoomSliderChanged(self):
        zoom = self.zoomSlider.value()
        if zoom != self.zoomSpinBox.value():
            self.controller.set_zoom(zoom)
            self.zoomSpinBox.setValue(zoom)

    def onZoomSpinBoxChanged(self):
        zoom = self.zoomSpinBox.value()
        if zoom != self.zoomSlider.value():
            self.controller.set_zoom(zoom)
            self.zoomSlider.setValue(zoom)

    def onFocusSliderChanged(self):
        focus = self.focusSlider.value()
        if focus != self.focusSpinBox.value():
            self.controller.set_focus(focus)
            self.focusSpinBox.setValue(focus)

    def onFocusSpinBoxChanged(self):
        focus = self.focusSpinBox.value()
        if focus != self.focusSlider.value():
            self.controller.set_focus(focus)
            self.focusSlider.setValue(focus)

    def onGainSliderChanged(self):
        gain = self.gainSlider.value()
        if gain != self.gainSpinBox.value():
            self.controller.set_gain(gain)
            self.gainSpinBox.setValue(gain)

    def onGainSpinBoxChanged(self):
        gain = self.gainSpinBox.value()
        if gain != self.gainSlider.value():
            self.controller.set_gain(gain)
            self.gainSlider.setValue(gain)

    def onExposureSliderChanged(self):
        exposure = self.exposureSlider.value()
        if exposure != self.exposureSpinBox.value():
            self.controller.set_exposure(exposure)
            self.exposureSpinBox.setValue(exposure)

    def onExposureSpinBoxChanged(self):
        exposure = self.exposureSpinBox.value()
        if exposure != self.exposureSlider.value():
            self.controller.set_exposure(exposure)
            self.exposureSlider.setValue(exposure)

    def onfpsSliderChanged(self):
        fps = self.fpsSlider.value()
        if fps != self.fpsSpinBox.value():
            self.controller.set_FPS(fps)
            self.fpsSpinBox.setValue(fps)

    def onfpsSpinBoxChanged(self):
        fps = self.fpsSpinBox.value()
        if fps != self.fpsSlider.value():
            self.controller.set_FPS(fps)
            self.fpsSlider.setValue(fps)
