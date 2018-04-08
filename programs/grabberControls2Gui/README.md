# Usage 

To use the grabberControls2GuiGUI you will need to be already running an instance of yarpserver and AravisGigE device. Once both are up and running, you can simply call the grabberControls2GuiGUI:

```bash
grabberControls2GuiGUI
```

By default it will try to connect to `/grabber`. If the port for the AravisGigE device is not `/grabber`, you can specify it when launching the `grabberControls2GuiGUI`:

```bash
grabberControls2GuiGUI --remote-port /whatever_port_you_want
```


## Useful links

* [yarp-devices-tricks](/doc/yarp-devices-tricks.md)
