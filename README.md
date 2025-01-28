# Autonomous Control Embedded Systems

## Overview
This will be used for the esp32, which will handle microRos communication and higher level processing to communicate with other microcontollers such as an arduino nano or motor controllers etc. 

## Micro-Ros Agent setup using docker
we are using a docker image containing the micro ros agent from this repository: https://github.com/micro-ROS/docker this allows for serial an udp transports

### install docker on ubuntu
```sudo apt install docker.io -y```
### UDP4 transport docker setup
set ```TRANSPORT_WIFI``` in the hardware_config.h file and rebuild to enable wifi as a transport. Also the port and IP address need to be set.
```sudo docker run -it --net=host microros/micro-ros-agent:<Ros2 version> udp4 -p <Desired Port>```

to automatically start on boot add 

```--restart=always```
### Serial transport docker setup
set ```TRANSPORT_SERIAL``` in the hardware_config.h file to enable serial transport.

```sudo docker run -it --net=host --device=<serial port e.g. /dev/ttyUSB0> microros/micro-ros-agent:humble serial --dev <serial port> ```

to automatically start on boot add 

```--restart=always```
