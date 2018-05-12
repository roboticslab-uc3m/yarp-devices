# Installation from Source Code

This repository stores different YARP devices for different actual devices used in different robots. This huge variability in hardware sometimes makes it complicated to track the dependencies for each device you may want to compile and install. 

This guide aims to simplify as much as possible the process of finding out which are the dependencies you actually need to install for the devices you want to use.

## OS Requirements

Some of the devices require a specific OS version to work. This table tracks the OS requirements: [os-requirements.csv](os-requirements.csv)

## Common dependencies (Ubuntu)

Some dependencies must be installed for all the devices:

  - [Install CMake](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md)
  - [Install YARP](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md)

Additionally, this project depends on YCM to download and build external packages. Although this process is intended to run automatically during the CMake configuration phase, you may still want to install YCM and said packages by yourself. In that respect, refer to [Install YCM](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md) and to the installation guides of any package listed below:

- [color-debug](https://github.com/roboticslab-uc3m/color-debug)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-googletest.md).

## Specific dependencies

<<<<<<< HEAD
| Dependency | OneCanBusOneWrapper | TwoCanBusThreeWrappers | AravisGigE | CanBusControlboard | CanBusHico | CanBusPeak | CanBusSocket | CuiAbsolute | FakeJoint | Jr3 | LacqueyFetch | LeapMotionSensor* | ProximitySensorsClient | SpaceNavigator | TechnosoftIpos | TextilesHand | WiimoteSensor | AravisGigEController** |
| --- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| [Aravis 0.4](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-aravis.md) |  :x: |  :x: | :white_check_mark: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |  :x: | :x: | :x: |  :x: | :x: |  :x: |
| [hcanpci (kernel module)](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-hcanpci.md) | :x: | :x: | :x: | :x: | :white_check_mark: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |
| Python 3+ |  :x: |  :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |  :x: | :x: |  :x: | :x: | :x: | :white_check_mark: |
| [setuptools (pip3)](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-setuptools.md#install-setuptools-using-pip3) |  :x: |  :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |  :x: | :x: |  :x: | :x: | :x: | :white_check_mark: |
| [Custom yarp Python 3 bindings](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md#install-python-bindings-with-iframegrabbercontrols2-support) | :x: |  :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :white_check_mark: |
| [Spacenav](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-spacenav.md#install-spacenav-ubuntu) |  :x: |  :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :white_check_mark: |  :x: | :x: | :x: | :x: |
| [XWiimote](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-xwiimote.md#install-xwiimote-ubuntu) |  :x: |  :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :white_check_mark: | :x: |
| [Leap Motion SDK v2](https://developer.leapmotion.com/sdk/v2)*** |  :x: |  :x: | :x: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :white_check_mark: | :x: | :x: |  :x: | :x: | :x: | :x: |
=======
Some devices require specific dependencies to work that must be satisfied. Use the following table to locate and install the dependencies required for your concrete application: [specific-dependencies.csv](specific-dependencies.csv)
>>>>>>> fix-122-icanbus

\* CMake find modules are smart enough to locate the SDK files in the usual paths. We recommend unzipping the `LeapSDK` folder in `/opt/LeapSDK` (make sure it contains `include/` and `lib/` at the top level).

\*\* See specific install instructions in the installation section.

\*\*\* Download link requires registration.

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

* GrabberControls2Gui requires Python 3+ with custom yarp Python bindings installed. Currently, installing them is not an easy task (see [comment348230791@roboticslab-uc3m/yarp-devices:145](https://github.com/roboticslab-uc3m/yarp-devices/issues/145#issuecomment-348230791) and [roboticslab-uc3m/installation-guides:26](https://github.com/roboticslab-uc3m/installation-guides/issues/26)) but we expect this to change in the future.

* Setup.py should take care of automatically installing the remaining dependencies for AravisGigEController. But it sometimes returns an error trying to install PySide. In that case, you can try to install it from the software repository: `sudo apt install python3-pyside`

## Useful links

* Usage instructions for the different devices contained in this repository can be found in each corresponding subdirectory.
* [yarp-devices tricks](yarp-devices-tricks.md)
