# Position movement:

```c
uint8_t msg_start[]={0x1F,0x00}; // Start the movement.
//this->waitSequence(1);  // Not required.
uint8_t msg_stop[]={0x0F,0x00}; // Does not stop, but is required for next start.
```

# Yarp VOCAB rpc commands:
* check status:
```
[get] [icmd] [cmds]
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
[set] [veld]
[set] [vmo] 0 800
[set] [vmos] (800)
```

* got to torque mode
```
[set] [trqd]
```

# Edit .ini config files in Calc (Excel)
Click `Separated by space` and `Merge delimiters`.
```
#!/bin/sh
openoffice.org -calc launchManipulation.ini
# libreoffice -calc launchManipulation.ini
sed -i 's/\"//g' launchManipulation.ini
```
