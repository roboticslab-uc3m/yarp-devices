# grabberControls2Gui


## Requirements
Depends on:
- Python 3+
- [setuptools (pip3)](http://robots.uc3m.es/gitbook-installation-guides/install-setuptools.html#install-setuptools-using-pip3)
- [YARP with Python 3 bindings](http://robots.uc3m.es/gitbook-installation-guides/install-yarp.md#install-python-bindings-with-iframegrabbercontrols2-support)


## Usage 

To use the `grabberControls2GuiGUI` you will need to be already running an instance of yarpserver and [AravisGigE](/libraries/YarpPlugins/AravisGigE) device. Once both are up and running, you can simply call the `grabberControls2GuiGUI`:

```bash
grabberControls2GuiGUI
```

By default it will try to connect to `/grabber`. If the port for the [AravisGigE](/libraries/YarpPlugins/AravisGigE) device is not `/grabber`, you can specify it when launching the `grabberControls2GuiGUI`:

```bash
grabberControls2GuiGUI --remote-port /whatever_port_you_want
```
