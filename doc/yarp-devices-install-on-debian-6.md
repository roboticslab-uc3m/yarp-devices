## Installation from Source Code (Debian 6.0.10)

### Why Debian 6.0.10?

Your OS needs the following (Debian 6.0.10 is a good intermediate solution in versions, and Debian is additionally more stable than Ubuntu):
  - A kernel old enough for the HicoCAN kernel drivers.
  - An OS new enough for github (you need a recent git version) and YARP (you need a recent CMake version).

### The specifics: APT sources

- /etc/apt/sources.list
```bash
deb http://archive.debian.org/debian/ squeeze main non-free contrib
deb-src http://archive.debian.org/debian/ squeeze main non-free contrib

deb ftp://ftp.gnome.org/debian-backports/ squeeze-backports main
deb-src ftp://ftp.gnome.org/debian-backports/ squeeze-backports main
```
- /etc/apt/apt.conf.d/90ignore-release-date
```bash
Acquire::Check-Valid-Until "false";
```
- If the previous file is not set, you can simply run:
```bash
sudo aptitude -o Acquire::Check-Valid-Until=false update
```

### Install the Software

First install the depencencies:
  - [Install CMake 2.8.9 (Debian 6.0.10)](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md#install-cmake-289-debian-6010)
  - [Install YARP 2.3.68+ (Debian 6.0.10)](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md#install-yarp-2368-debian-6010)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/yarp-devices.git  # Download yarp-devices software from the repository
cd yarp-devices; mkdir build; cd build; cmake ..  # Configure the yarp-devices software
make  # Compile
sudo make install  # Install :-)
```

For additional TEO options use ccmake instead of cmake.

Finally, install the hcanpci kernel module:

 - [Install hcanpci kernel module (Debian 6.0.10)]( /doc/yarp-devices-install-hcanpci-on-debian-6.md )

You may also want the `xsensmtx` device:

 - [Install ICub and xsensmtx](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install_icub.md)

### Even more!

Done! You are now probably interested in one of the following links:
  - [yarp-devices - Now what can I do?]( /doc/yarp-devices-post-install.md )
