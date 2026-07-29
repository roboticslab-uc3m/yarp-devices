# Installation from Source Code

First install the dependencies:
- [Install CMake 3.19+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-ycm.md/)
- [Install YARP 3.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-yarp.md/)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-googletest.md/).

### Components with known additional/specific dependencies

- [../libraries/YarpPlugins/CanBusHico](../libraries/YarpPlugins/CanBusHico#requirements)
- [../libraries/YarpPlugins/CanBusPeak](../libraries/YarpPlugins/CanBusPeak#requirements)
- [../libraries/YarpPlugins/Jr3Pci](../libraries/YarpPlugins/Jr3Pci#requirements)
- [../libraries/YarpPlugins/LeapMotionSensor](../libraries/YarpPlugins/LeapMotionSensor#requirements)
- [../libraries/YarpPlugins/SpaceNavigator](../libraries/YarpPlugins/SpaceNavigator#requirements)
- [../libraries/YarpPlugins/Wiimote](../libraries/YarpPlugins/Wiimote#requirements)
- The following components additionally need some kind of CAN Bus driver (e.g. a [CanBusHico](../libraries/YarpPlugins/CanBusHico) or [CanBusPeak](../libraries/YarpPlugins/CanBusPeak)):
    - [../libraries/YarpPlugins/CanBusBroker](../libraries/YarpPlugins/CanBusBroker)
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

## Useful links

* Usage instructions for the different devices contained in this repository can be found in each corresponding subdirectory.
