from PySide import QtCore,QtGui
from PySide import QtUiTools
import os, sys

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

class AravisGigEControllerGUI(QtGui.QWidget):
    def __init__(self, controller=None, parent=None):
        QtGui.QWidget.__init__(self, parent)

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
        ui_file_path = os.path.join(os.path.realpath(os.path.dirname(__file__)), 'AravisGigEControllerGUI.ui')
        main_widget = load_ui(ui_file_path, self)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(main_widget)
        self.setLayout(layout)

        # Get a reference to all required widgets
        self.zoomSlider = self.findChild(QtGui.QSlider, 'zoomSlider')
        self.zoomSpinBox = self.findChild(QtGui.QDoubleSpinBox, 'zoomSpinBox')
        self.focusSlider = self.findChild(QtGui.QSlider, 'focusSlider')
        self.focusSpinBox = self.findChild(QtGui.QDoubleSpinBox, 'focusSpinBox')
        self.gainSlider = self.findChild(QtGui.QSlider, 'gainSlider')
        self.gainSpinBox = self.findChild(QtGui.QDoubleSpinBox, 'gainSpinBox')
        self.exposureSlider = self.findChild(QtGui.QSlider, 'exposureSlider')
        self.exposureSpinBox = self.findChild(QtGui.QDoubleSpinBox, 'exposureSpinBox')
        self.fpsSlider = self.findChild(QtGui.QSlider, 'fpsSlider')
        self.fpsSpinBox = self.findChild(QtGui.QDoubleSpinBox, 'fpsSpinBox')

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


if __name__ == '__main__':
    from AravisGigEController import AravisGigEController

    # Create Qt app
    app = QtGui.QApplication(sys.argv)

    # Create the widget and show it
    controller = AravisGigEController()
    gui = AravisGigEControllerGUI(controller)
    gui.show()

    # Run the app
    sys.exit(app.exec_())