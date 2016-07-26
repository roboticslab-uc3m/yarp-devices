## Installation from Source Code (Debian 6.0)

### Why Debian 6.0?

Your OS needs the following (Debian 6.0 is a good intermediate solution in versions, and Debian is additionally more stable than Ubuntu):
  - A kernel old enough for the HicoCAN kernel drivers.
  - An OS new enough for github (you need a recent git version) and YARP (you need a recent CMake version).

### Install the Software

First install the depencencies:
  - [Install CMake (Debian 6.0)]( /doc/teo_body_install_cmake_on_debian_6.md )
  - [Install YARP (Debian 6.0)]( /doc/teo_body_install_yarp_on_debian_6.md )

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/teo-body.git  # Download teo-body software from the repository
cd teo-body; mkdir build; cd build; cmake ..  # Configure the teo-body software
make  # Compile
sudo make install  # Install :-)
```

For additional TEO options use ccmake instead of cmake.

Finally, install the hcanpci kernel module:

  - [Install hcanpci kernel module (Debian 6.0)]( /doc/teo_body_install_hcanpci_on_debian_6.md )

### Even more!

Done! You are now probably interested in one of the following links:
  - [teo-body - Now what can I do?]( /doc/teo_body_post_install.md )
  - teo_body_environment_variables
