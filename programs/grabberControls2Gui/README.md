# grabberControls2Gui


## Requirements
Depends on:
- Python 3+
- [setuptools using pip3](https://robots.uc3m.es/installation-guides/install-setuptools.html#install-setuptools-using-pip3)
- [YARP with Python 3 bindings](https://robots.uc3m.es/installation-guides/install-yarp.html#install-python-bindings)


## Usage

To use the `grabberControls2GuiGUI` you will need to be already running an instance of yarpserver and [AravisGigE](/libraries/YarpPlugins/AravisGigE) device. Once both are up and running, you can simply call the `grabberControls2GuiGUI`:

```bash
grabberControls2GuiGUI
```

By default it will try to connect to `/grabber`. If the port for the [AravisGigE](/libraries/YarpPlugins/AravisGigE) device is not `/grabber`, you can specify it when launching the `grabberControls2GuiGUI`:

```bash
grabberControls2GuiGUI --remote-port /whatever_port_you_want
```
