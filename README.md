# Autonomous Control Embedded Systems

## Overview
This will be used for the esp32, which will handle microRos communication and higher level processing to communicate with other microcontollers such as an arduino nano or motor controllers etc. 
## ROS2 integration
### messages
We are using ackermann messages to send and recive data about the car. 


## Micro-Ros Agent setup using docker
we are using a docker image containing the micro ros agent from this repository: https://github.com/micro-ROS/docker this allows for serial an udp transports

### Install Docker on Ubuntu
```sudo apt install docker.io -y```
### UDP4 Transport Docker Setup
set ```TRANSPORT_WIFI``` in the hardware_config.h file and rebuild to enable wifi as a transport. Also the NET_SSID, NET_PASS, uROS_PORT and uROS_IP macros need to be set in the uRos_fns.h file.
```sudo docker run -it --net=host --privileged microros/micro-ros-agent:<Ros2 version> udp4 -p <Desired Port>```

to automatically start on boot add: ```--restart=always```
### Serial Transport Docker Setup
set ```TRANSPORT_SERIAL``` in the hardware_config.h file to enable serial transport.

```sudo docker run -it --net=host --privileged --device=<serial port e.g. /dev/ttyUSB0> microros/micro-ros-agent:humble serial --dev <serial port> ```

to automatically start on boot add:  ```--restart=always```
