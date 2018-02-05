# Canbus commands:

Position movement:
```c
uint8_t msg_start[]={0x1F,0x00}; // Start the movement.
//this->waitSequence(1);  // Not required.
uint8_t msg_stop[]={0x0F,0x00}; // Does not stop, but is required for next start.
```

# Yarp RPC commands (VOCABs):

Note that this is a hack. VOCABs may be updated without warning. The recommended YARP-ish way is via YARP_dev interfaces. An interactive way to do this is via an `ipython` console and following this repository's [Python examples](https://github.com/roboticslab-uc3m/yarp-devices/tree/develop/example/python).

## remote_controlboard
* check status:
```
[get] [icmd] [cmds]
```

* get limits:
```
[get] [llim]
```

* get velocity limits:
```
[get] [vlim]
```

* check if pos motion done:
```
[get] [don] 0
```

* stop all:
```
[set] [stos]
```

* set ref velocities (for pos mode)
```
[set] [vel] 0 10.0
[set] [vels] (10.0)
```

* set ref accelerations (for pos mode)
```
[set] [acc] 0 10.0
[set] [accs] (10.0)
```

* go to vel mode, move
```
[set] [icmd] [cmod] 0 [vel]
[set] [vmos] (800)
```

* go to torque mode
```
[set] [icmd] [cmod] 0 [torq]
```

# Edit .ini config files in Calc (Excel)
Click `Separated by space` and `Merge delimiters`.
```bash
#!/bin/sh
openoffice.org -calc launchManipulation.ini
# libreoffice -calc launchManipulation.ini
sed -i 's/\"//g' launchManipulation.ini
```
