## Install CMake (Debian 6.0)

We use CMake for project generating. To install the version of CMake required for teo-body on Debian 6.0, you must activate the distribution's backports. Add:

```bash
deb http://backports.debian.org/debian-backports/ squeeze-backports main
```

to your /etc/apt/sources.list file. Then run the following lines from a terminal. Note that you will be prompted for your password upon using '''sudo'''.

```bash
sudo apt-get update
sudo apt-get install -t squeeze-backports cmake cmake-curses-gui
```

