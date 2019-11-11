
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

In terminal A:
``` bash
roscore
```

In terminal B (all the rest part):
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

## TF Frames

Run commands in separate terminals:

``` bash
rosrun beginner_tutorials talker
```
Inspect tf frames:
``` bash
rosrun tf tf_echo /world /talk
```
To inspect the visualized graph:
``` bash
rosrun rqt_tf_tree rqt_tf_tree 
```
To save graph as pdf:
``` bash
rosrun tf view_frames
```

## ROSTest

First run the talker node:

``` bash
rosrun beginner_tutorials talker
```
To compile and run the tests:

``` bash
catkin_make run_tests_beginner_tutorials
```
After complie completes, run:
``` bash
rostest beginner_tutorials test.launch
```

or

``` bash
rosrun beginner_tutorials test_talker 
```

## ROS Bag

Use ROS bag to record topics info:
``` bash
roslaunch beginner_tutorials TalkAndListen.launch record:=enable
```

To inspect recorded ROS bag info:
``` bash
rosbag info talker.bag
```

To replay the recorded ROS bag, first run listener node:
``` bash
rosrun beginner_tutorials listener
```

Then start the replay:
``` bash
rosbag play talker.bag
```

