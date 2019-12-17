# Canbus commands

Position movement:
```c
uint8_t msg_start[]={0x1F,0x00}; // Start the movement.
//this->waitSequence(1);  // Not required.
uint8_t msg_stop[]={0x0F,0x00}; // Does not stop, but is required for next start.
```

# Yarp RPC commands (VOCABs)

Note that this is a hack. VOCABs may be updated without warning. The recommended YARP-ish way is via YARP_dev interfaces. An interactive way to do this is via an `ipython` console and following this repository's [Python examples](../examples/python).

## remote_controlboard

### encoder commands
* query encoder reads:
```
[get] [enc] 0
[get] [encs]
```

* get estimated instantaneous speeds:
```
[get] [esp] 0
[get] [esps]
```

* get estimated instantaneous accelerations:
```
[get] [eac] 0
[get] [eacs]
```

### control modes
* get control modes:
```
[get] [icmd] [cmod] 0
[get] [icmd] [cmds]
[get] [icmd] [cmog] (0 2 4)
```

* set pos mode
```
[set] [icmd] [cmod] 0 [pos]
[set] [icmd] [cmds] ([pos] [pos] [pos] [pos] [pos] [pos])
[set] [icmd] [cmog] (0 2 4) ([pos] [pos] [pos])
```

* set vel mode
```
[set] [icmd] [cmod] 0 [vel]
[set] [icmd] [cmds] ([vel] [vel] [vel] [vel] [vel] [vel])
[set] [icmd] [cmog] (0 2 4) ([vel] [vel] [vel])
```

* set torque mode
```
[set] [icmd] [cmod] 0 [torq]
[set] [icmd] [cmds] ([torq] [torq] [torq] [torq] [torq] [torq])
[set] [icmd] [cmog] (0 2 4) ([torq] [torq] [torq])
```

* set current mode (same as torque mode in TechnosoftIpos)
```
[set] [icmd] [cmod] 0 [icur]
[set] [icmd] [cmds] ([icur] [icur] [icur] [icur] [icur] [icur])
[set] [icmd] [cmog] (0 2 4) ([icur] [icur] [icur])
```

### in pos mode
* set position
```
[set] [pos] 0 10.0
[set] [poss] (10.0 10.0 10.0 10.0 10.0 10.0)
[set] [posg] 3 (0 2 4) (10.0 10.0 10.0)
```

* set ref velocities
```
[set] [vel] 0 10.0
[set] [vels] (10.0 10.0 10.0 10.0 10.0 10.0)
[set] [velg] 3 (0 2 4) (10.0 10.0 10.0)
```

* set ref accelerations
```
[set] [acc] 0 10.0
[set] [accs] (10.0 10.0 10.0 10.0 10.0 10.0)
[set] [accg] 3 (0 2 4) (10.0 10.0 10.0)
```

* check if motion done:
```
[get] [don] 0
[get] [dons]
[get] [dong] 3 (0 2 4)
```

* command stop:
```
[set] [sto] 0
[set] [stos]
[set] [stog] (0 2 4)
```

### in vel mode
* move in vel mode
```
[set] [vmos] (5.0 5.0 5.0)
```

### in torq mode
* get actual torque
```
[get] [torq] [trq] 0
```

* set reference torque
```
[set] [torq] [ref] 0 1.0
```

### in icur mode
* get reference current
```
[get] [icur] [ref] 0
```

### limits
* get pos limits:
```
[get] [llim] 0
```

* get vel limits:
```
[get] [vlim] 0
```

### remote calibrator
* homing:
```
[set] [reca] [hom] 0
[set] [reca] [homs]
```

## analogsensorClient

### calibration
* calibrate channel (single sensor)
```
[iana] [calc] 0
```

* calibrate all sensors
```
[iana] [cal]
```


# Edit .ini config files in Calc (Excel)
Click `Separated by space` and `Merge delimiters`.
```bash
#!/bin/sh
openoffice.org -calc launchManipulation.ini
# libreoffice -calc launchManipulation.ini
sed -i 's/\"//g' launchManipulation.ini
```
