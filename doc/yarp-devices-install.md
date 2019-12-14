# Installation from Source Code

This repository stores different YARP devices for different actual devices used in different robots. This huge variability in hardware sometimes makes it complicated to track the dependencies for each device you may want to compile and install.

This guide aims to simplify as much as possible the process of finding out which are the dependencies you actually need to install for the devices you want to use.

## OS Requirements

Some of the devices require a specific OS version to work. This table tracks the OS requirements: [os-requirements.csv](os-requirements.csv)

## Common dependencies (Ubuntu)

Some dependencies must be installed for all the devices:

- [Install CMake 3.5+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.1+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install color-debug](https://github.com/roboticslab-uc3m/color-debug)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-googletest.md/).

## Specific dependencies

Some devices require specific dependencies to work that must be satisfied. Use the following table to locate and install the dependencies required for your concrete application: [specific-dependencies.csv](specific-dependencies.csv)

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

* GrabberControls2Gui requires Python 3+ with custom yarp Python bindings installed. Currently, installing them is not an easy task (see [comment348230791@roboticslab-uc3m/yarp-devices#145](https://github.com/roboticslab-uc3m/yarp-devices/issues/145#issuecomment-348230791) and [roboticslab-uc3m/installation-guides#26](https://github.com/roboticslab-uc3m/installation-guides/issues/26)) but we expect this to change in the future.

* Setup.py should take care of automatically installing the remaining dependencies for AravisGigEController (`sudo python3 setup.py install`).

## Useful links

* Usage instructions for the different devices contained in this repository can be found in each corresponding subdirectory.
* [yarp-devices tricks](yarp-devices-tricks.md)
