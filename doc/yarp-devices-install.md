# Installation from Source Code

First install the dependencies:
- [Install CMake 3.12+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.3+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install color-debug](https://github.com/roboticslab-uc3m/color-debug)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-googletest.md/).

### Components with known additional/specific dependencies

- [../libraries/YarpPlugins/AmorControlboard](../libraries/YarpPlugins/AmorControlboard#requirements)
- [../libraries/YarpPlugins/AravisGigE](../libraries/YarpPlugins/AravisGigE#requirements)
- [../libraries/YarpPlugins/CanBusHico](../libraries/YarpPlugins/CanBusHico#requirements)
- [../libraries/YarpPlugins/CanBusPeak](../libraries/YarpPlugins/CanBusPeak#requirements)
- [../libraries/YarpPlugins/Jr3](../libraries/YarpPlugins/Jr3#requirements)
- [../libraries/YarpPlugins/LeapMotionSensor](../libraries/YarpPlugins/LeapMotionSensor#requirements)
- [../libraries/YarpPlugins/SpaceNavigator](../libraries/YarpPlugins/SpaceNavigator#requirements)
- [../libraries/YarpPlugins/WiimoteSensor](../libraries/YarpPlugins/WiimoteSensor#requirements)
- [../programs/grabberControls2Gui](../programs/grabberControls2Gui#requirements)
- The following components additionally need some kind of CAN Bus driver (e.g. a [CanBusHico](../libraries/YarpPlugins/CanBusHico) or [CanBusPeak](../libraries/YarpPlugins/CanBusPeak)):
    - [../libraries/YarpPlugins/CanBusControlboard](../libraries/YarpPlugins/CanBusControlboard)
    - [../libraries/YarpPlugins/CuiAbsolute](../libraries/YarpPlugins/CuiAbsolute)
    - [../libraries/YarpPlugins/FakeJoint](../libraries/YarpPlugins/FakeJoint)
    - [../libraries/YarpPlugins/LacqueyFetch](../libraries/YarpPlugins/LacqueyFetch)
    - [../libraries/YarpPlugins/TechnosoftIpos](../libraries/YarpPlugins/TechnosoftIpos)
    - [../libraries/YarpPlugins/TextilesHand](../libraries/YarpPlugins/TextilesHand)


## Installation (Ubuntu)

Once the required dependencies have been installed, the code has to be compiled and installed. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/yarp-devices.git  # Download yarp-devices software from the repository
cd yarp-devices; mkdir build; cd build; cmake ..  # Configure the yarp-devices software
make -j$(nproc)  # Compile
sudo make install  # Install :-)
sudo ldconfig  # Just in case
```

Remember to enable the devices you want to compile using `ccmake` instead of `cmake`.

### Useful info to install GrabberControls2Gui

When installing GrabberControls2Gui, take into account the following points:

* GrabberControls2Gui requires Python 3+ with custom yarp Python bindings installed. Currently, installing them is not an easy task (see [comment348230791@roboticslab-uc3m/yarp-devices#145](https://github.com/roboticslab-uc3m/yarp-devices/issues/145#issuecomment-348230791) and [roboticslab-uc3m/installation-guides#26](https://github.com/roboticslab-uc3m/installation-guides/issues/26)) but we expect this to change in the future.

* Setup.py should take care of automatically installing the remaining dependencies for AravisGigEController (`sudo python3 setup.py install`).

## Useful links

* Usage instructions for the different devices contained in this repository can be found in each corresponding subdirectory.
