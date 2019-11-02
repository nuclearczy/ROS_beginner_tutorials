# ROS_beginner_tutorials

[![Build Status](https://travis-ci.com/nuclearczy/ROS_beginner_tutorials.svg?branch=master)](https://travis-ci.com/nuclearczy/ROS_beginner_tutorials)
![GitHub](https://img.shields.io/github/license/nuclearczy/ROS_beginner_tutorials)

Author: Zuyang Cao

## Overview

A simple publisher and subscriber tutorial on ROS. The publisher node can take input from launch file and the subscriber has a service to change the listener status.

## Dependencies and Environments

- Ubuntu 16.04 LTS
- ROS-kinetic-full

## Installation Steps

``` bash
cd ~/catkin_ws/src
git clone https://github.com/nuclearczy/ROS_beginner_tutorials
cd ..
catkin_make
source devel/setup.bash
```

## Run the Code

``` bash
roslaunch beginner_tutorials TalkAndListen.launch
```

### Modify Talker's Name

You can modify the talker's name when using ROS launch:

``` bash
roslaunch beginner_tutorials TalkAndListen.launch talker_name:=<name here>
```

### Modify Message

You can modify the message when using ROS launch:

``` bash
roslaunch beginner_tutorials TalkAndListen.launch message_content:=<message here>
```

## Calling ROS Service

After launched the listener_node, a toggle_listen_status service can be called to change the listener_node working status:

``` bash
rosservice call /toggle_listen_status
```


