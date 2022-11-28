# itm_rasp_mobile_robot

This package is for controlling the ITM mobile robots on a Raspberry Pi onboard computer with Ubuntu OS. This branch is available for ROS 2, if you want to use the codes for ROS 1. Please change to the **ros1** branch.

## Communicating through UART

Based on the work from Dr. Ebel and Mr. Rosenfelder, one can directly communicate with the onboard computer Beaglebone on the ITM mobile robots through UART. The ROS commands can be transferred through the UART without any special packages on the Beaglebone. One can launch the program

```bash
ros2 launch itm_rasp_mobile_robot cmd_to_serial.launch.py
```

Two arguments in the configure file **config/ros_cmd_uart.yaml** need to be specified:

- port: which UART port of the Raspberry Pi is to be used. Note that, in some cases, the UART port should be authorized.
- sub_topic: subscribed topic in ROS, which is to be transferred through the UART connection.


## Open the onboard USB Camera

In the practice of **HERA Motion** one may need to subscribe the message from camera. The configuration are specified in the configure file in the folder **config**.

For instance, you can launch a camera node

```bash
ros2 launch itm_rasp_mobile_robot usb_cam.launch.py
```

Note that in the **config** folder there are several configure files for camera experiments. One can use **usb_cam** package to control the USB camera that is attached on the Raspberry Pi.
