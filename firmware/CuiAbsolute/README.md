# Cui Absolute Firmware

## Necessary requirements

First, we need to install the MPLAB tool v.8.92. If we already have windows installed on our computer, we can install it by directly downloading the application in [this](https://drive.google.com/open?id=1n51XC7JwGAtncWq1itDxE7FpyqdSyaWD) link.
In case of working with Ubuntu or another S.O, you can download VirtualBox and load the virtual machine contained in the following [link](https://drive.google.com/open?id=0BxR76I90oKSmQ0xsS0loZVhhRnM). This virtual machine contains all the necessary tools installed on Windows 7: **MPLAM IDE v.8.92** and **EasySetup**.

## How to download the firmware on the PIC

Once MPLAB is opened, we load the project by selecting *Project > Open* and select the file `pic_source.mcp`
This will open the work environment, where `main.c` is the code file that will be compiled in our PIC. 
Make sure that the MPLAB is already connected to the PIC. We need external power connected to the programmer. It's recommended to disconnect the CAN wire that is connected to the driver so as not to derive electric current to it.

First, we must modify [line 69](./Pic_source/main.c#L69), corresponding to the ID of that encoder. We can see the correspondence in the [diagram](http://robots.uc3m.es/gitbook-teo-developer-manual/diagrams.html#joint-indexes) and adding 100 to the ID of the joint. Example: for the elbow of the left arm joint ID 24 we should write [ID 124](./Pic_source/main.c#L69).
Then we need to follow the next steps:
* We compile: `Project> Build All`
* Select the programmer: `Programmer> Select Programmer> MPLAB ICD 2`
* Connect the programmer to the PIC: `Programmer> Connect`
* Program: `Porgrammer> Program`

## Set the CUI to zero

In this case we'll use the PCAN-View program. This is a tool included in the [Peak-CAN driver](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-pcan.md) (test utilities).
The first window you can see is the `Connect` window. You need to choose the CAN channel in which the CUI is connected. Below it's important to set the `Bitrate to 1Mbps (1000000 bps)`.
Then the reception and transmission window will be opened. In the transmission window we can see the last messages saved to send. If the message hasn't been created before, we must create it by selecting `Transmit> New Message`.
We can set the encoder ID in decimal format by selecting `File> Settings> CAN ID Format> Decimal`. 
Once the message window is opened with `New Message` or `Edit Message` you need to write or modify the ID to send the message by double clicking on the message or by pressing Enter on it. 
Fields `Len: 1` and `Data: ff`. The rest of the fields must be unchecked, except `Paused`. Then you can press `Ok`. To confirm that it has been done correctly, we must receive a message in the Rx window with ID = OP (0x200) + canID. It will be an ACK message.

Another way to read the value of the CUI will be by sending a pull message with `Data: 03` and the same ID. We should get a message with **zeros** in the Data field of the received message and ID = OP(0x180) + canID.
