---
title: This Repo
layout: template
filename: this_repo
--- 

# What is in this repository

This repository contains code for the Olin College, Fundamentals of Robotics class, but the intro folder contains some simple examples of ROS code. To use code in this repository, fork the repository and clone it into the `src` folder of your catkin workspace.

## Intro Folder
The intro folder is a ROS package demonsrating a few different things you can do with ROS. All of the examples are duplicated in python (scripts folder) and c++ (src folder).

### `src` Folder
This folder contains three scripts which publish and subscirbe to a few topics:

- `hello_world.cpp` is a copy of the hello world script from the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29), publishing a hello world message to the topic `/hello_world`. To run this script, use `rosrun intro talker`.
- `hello_echo.cpp` demonstrates a subscriber and publisher in the same script. It listends to the `/hello_world` topic and echos the messages it hears to the `/echo_world` topic. Run this code using `rosrun intro echo`. This node also publishes a status message to `/echo_status` every two seconds, so you can check that the code is running without publishing anything to the `/hello_world` topic. In this code, the second message is published from inside the callback function from the first message.
- `hello_counter.cpp` is a little different from `hello_echo.cpp` because it publishes a message from the main run loop, and the callback function just updates a variable. This script subscribes to `/hello_world` and counts the number of messages it has received. It then periodically publishes a count of how many messages it has received to `/message_count`. run this using `rosrun intro counter`.

These three pieces of code can be run together to move information around: if you run all three at once you will see hello world messages on the `/hello_world` and `/hello_echo` topics, as well as a count of the number of messages published to `/hello_world` on the `/message_count` topic. However, you can also see how ROS's modularity can make it easy to test individual pieces of this code. You can run the echo node by itself and publish messages to `/hello_world` from the terminal, then those messages will be echoed on the topic `/echo_world`, or you can run the counter node by itself, or with the echo node, and you, again, can publish messages to `/hello_world`, and test just the counter node.

### `scripts` Folder
This folder contains scripts with the same functionality as the src folder, except implemented in python.

- `hello_world.py`: publishes to `/hello_world`. run with `rosrun intro hello_world.py`
- `hello_echo.py`: echos messages from `/hello_world` to `/echo_world`. run with `rosrun intro hello_echo.py`
- `base_ROS_coms.py`: counts messages on `/hello_world` and publishes count to `/message_count`. run with `rosrun intro base_ROS_coms.py`

### `launch` Folder
This contains two very simple examples of launch files, one for the c++ implementatoins and one for the python implementations. In each case, the launch file runs all three scripts described above. Note: you don't need to run roscore before using a roslaunch, roscore will be run automatically if it is not already running.