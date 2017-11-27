# Usage 

## AravisGigE device
To use this YARP device a GigE camera is required. Once connected, use the following commands to control the camera and receive images.

### Running the device
To run the device and connect to the camera, simply run:

`$ yarpdev --name AravisGigE`

If you want to test the device without an actual camera, you can use a fake Aravis camera:

`$ yarpdev --name AravisGigE --fake`

### Obtaining a color image from the device
This YARP device returns a raw 8-bit image from the camera, to obtain a color image from the image, the stream has to be connected using the [Bayer carrier ](http://www.yarp.it/carrier_config.html#carrier_config_bayer) to interpret the raw image as a RGB image. Given an `AravisGigE` device named `/grabber` and an input port named `/v` (from a viewer, for instance), the command to run to connect them is:

`$ yarp connect /grabber /v udp+recv.bayer+order.gbrg` 

### Camera parameters control
The control of the camera parameters is performed from the image port (`/grabber` by default ), through a [RPC interface](http://www.yarp.it/rpc_ports.html).

`$ yarp rpc /grabber`

Once there one can send command to the camera. The most common commands are: `has`, `get` and `set`. 

#### has
With the `has` command one can query the device if it has some property. For instance, to check if the camera has zoom controls:

`fgc2 has feat 16`

#### get
With the ' get' command one can query the *value* of some property. For instance, to check the current gain value:

`fgc2 get feat 9`

#### set 
With the `set` command one can change the *value* of some property. For instance, to set the zoom to the maximum value:

`fgc2 set feat 16 100`

#### Available features
These are the features currently available in YARP. To check which ones are supported by the camera, the `has` command can be used:

| Feature | Enum name | Enum value |
| --- | --- | ---| 
| Brightness | YARP_FEATURE_BRIGHTNESS | 0 |
| Exposure |YARP_FEATURE_EXPOSURE | 1 |
| Sharpness | YARP_FEATURE_SHARPNESS | 2 |
| White Balance | YARP_FEATURE_WHITE_BALANCE | 3 |
| Hue | YARP_FEATURE_HUE | 4 |
| Saturation | YARP_FEATURE_SATURATION | 5 |
| Gamma |YARP_FEATURE_GAMMA | 6 |
| Shutter |YARP_FEATURE_SHUTTER | 7 |
| Gain |YARP_FEATURE_GAIN | 8 |
| Iris |YARP_FEATURE_IRIS | 9 |
| Focus |YARP_FEATURE_FOCUS | 10 |
| Temperature |YARP_FEATURE_TEMPERATURE | 11 |
| Trigger |YARP_FEATURE_TRIGGER | 12 |
| Trigger delay |YARP_FEATURE_TRIGGER_DELAY | 13 |
| White Shading |YARP_FEATURE_WHITE_SHADING | 14 |
| Frame Rate |YARP_FEATURE_FRAME_RATE | 15 |
| Zoom | YARP_FEATURE_ZOOM | 16 |
| Pan |YARP_FEATURE_PAN | 17 |
| Tilt |YARP_FEATURE_TILT | 18 |
| Optical Filter |YARP_FEATURE_OPTICAL_FILTER | 19 |
| Capture size | YARP_FEATURE_CAPTURE_SIZE | 20 |
| Capture quality | YARP_FEATURE_CAPTURE_QUALITY | 21 |
| Mirror | YARP_FEATURE_MIRROR | 22 |
| Number of features | YARP_FEATURE_NUMBER_OF | 23 |

### FAQ
#### I can receive an image, but it is all dark, what can I do?

This is probably due to a bad configuration of the camera parameters. Try to increase the gain or exposure until the image starts looking brighter. For the 1.0.B06 lab, some values that work great are:

```
Gain: 10
Exposure: 32000
````

#### I cannot receive a color image, but I receive a grey image with a regular point pattern on it.

What you are receiving is the raw image of the camera. To obtain a color image from it you need to decode it using a Bayer filter. Follow the steps in the section [Obtaining a color image from the device](https://github.com/roboticslab-uc3m/yarp-devices/blob/develop/doc/yarp-devices-usage.md#obtaining-a-color-image-from-the-device) in this very same guide to fix it.