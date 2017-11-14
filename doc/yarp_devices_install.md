# Installation from Source Code

This repository stores different YARP devices for different actual devices used in different robots. This huge variability in hardware sometimes makes it complicated to track the dependencies for each device you may want to compile and install. 

This guide aims to simplify as much as possible the process of finding out which are the dependencies you actually need to install for the devices you want to use.

## OS Requirements

Some of the devices require a specific OS version to work. This table tracks the OS requirements:

* :white_check_mark: The device works in this OS.
* :x:  The device won't work on this OS.
* :grey_question: The device hasn't been tested on this OS yet.


| OS | OneCanBusOneWrapper | TwoCanBusThreeWrappers | AravisGigE | CanBusControlboard | CanBusHico | CuiAbsolute | FakeJoint | Jr3 | LacqueyFetch | LeapMotionSensor | ProximitySensorsClient | SpaceNavigator | TechnosoftIpos | TextilesHand | WiimoteSensor |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| [Debian 6.0.10](yarp_devices_install_on_debian_6.md) | :white_check_mark: | :white_check_mark: | :grey_question: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: |
| Ubuntu 14.04 (and derivatives) | :grey_question: |  :grey_question: | :white_check_mark: |  :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: |  :grey_question: | :grey_question: | :grey_question: | 
| Ubuntu 16.04 (and derivatives) | :grey_question: |  :grey_question: | :white_check_mark: |  :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | :grey_question: | 


## Common dependencies

Some dependencies must be installed for all the devices:

  - [Install CMake 2.8.9 (Debian 6.0.10)](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md#install-cmake-289-debian-6010)
  - [Install YARP 2.3.68+ (Debian 6.0.10)](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md#install-yarp-2368-debian-6010)

## Specific dependencies

Some devices require specific dependencies to work that must be satisfied. Use the following table to locate and install the dependencies required for your concrete application.

| Dependency | OneCanBusOneWrapper | TwoCanBusThreeWrappers | AravisGigE | CanBusControlboard | CanBusHico | CuiAbsolute | FakeJoint | Jr3 | LacqueyFetch | LeapMotionSensor | ProximitySensorsClient | SpaceNavigator | TechnosoftIpos | TextilesHand | WiimoteSensor |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| [Aravis 0.4]() |  :x: |  :x: | :white_check_mark: |  :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |  :x: | :x: | :x: | 
| [hcanpci (kernel module)](yarp_devices_install_on_debian_6.md) | :white_check_mark: | :white_check_mark: | :x: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: | :white_check_mark: |


## Installation

Once the required dependencies have been install, the code has to be compiled and installed. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/yarp-devices.git  # Download yarp-devices software from the repository
cd yarp-devices; mkdir build; cd build; cmake ..  # Configure the yarp-devices software
make  # Compile
sudo make install  # Install :-)
```

Remember to enable the devices you want to compile using `ccmake` instead of `cmake`.

## Even more!

Done! You are now probably interested in one of the following links:
  - [yarp-devices - Now what can I do?]( /doc/yarp_devices_post_install.md )
