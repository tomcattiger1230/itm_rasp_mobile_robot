# itm_rasp_mobile_robot

This package is for controlling the ITM mobile robots on a Raspberry Pi onboard computer with Ubuntu OS. This branch is available for ROS 1, if you want to use the codes for ROS 2. Please change to the master branch.

## Communicating through UART

Based on the work from Dr. Ebel and Mr. Rosenfelder, one can directly communicate with the onboard computer Beaglebone on the ITM mobile robots through UART. The ROS commands can be transferred through the UART without any special packages on the Beaglebone. One can launch the program

```bash
roslaunch itm_rasp_mobile_robot cmd_serial_converter.launch
```

Two arguments need to be specified:

- port: which UART port of the Raspberry Pi is to be used. Note that, in some cases, the UART port should be authorized.
- sub_topic: subscribed topic in ROS, which is to be transferred through the UART connection.

> Note that in the **config** folder there are several configure files for camera experiments. One can use **usb_cam** package to control the USB camera that is attached on the Raspberry Pi.
