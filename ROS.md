---
title: ROS Intro
layout: template
filename: ROS
--- 

# Introduction to ROS
This page is a high level introduction to what ROS is, and how you will use it. Going through the tutorials ROS provides is actually a good way to learn how to use ROS, but they tend to dig into the details of how to do things without telling you what the point is.
In [the tutorials page](tutorials), I give a quick summary of the point behind each of the first handful of tutorials.

### contents
- [Introduction - what is this page about](#introduction-to-ros)
- [What is ROS - high level overview of ROS](#what-is-ros)
- [How Does It Work - high level overview of ROS communication](#ok-that-makes-some-conceptual-sense-but-how-does-it-work)

## What is ROS?

### Communication
ROS (Robot Operating System) is primarily a framework for managing communication between different portions of your code. There are a few reasons that this is extremely important:

-  It makes it for different pieces of code to share pretty much any type of information with very little setup work from the coder.
- It standardizes message types coming from sensors such as cameras or laser scanners, and therefore abstracts out the physical sensor
  - Once you write code to react to one camera, you can change to and entirely different camera and that same code will still work.
- ROS communication can handle communication within one computer, or across multiple machines connected to the same network
  - If you have a computer on your robot, and you want to monitor it from your computer, it is very easy to get access to any information that is sent between portions of the code.

### Built in Functionality
In addition to managing communication, ROS has a large number of preexisting packages which will help you complete your project. One of the stated goals of ROS is to encourage and support reusing code across multiple robotics projects. As a result, you can find a lot of small pieces of code to do things like:
- read a sensor and publish the input in a standard ROS message format
- Run a PID control loop or Kalman filter
- Perform common tasks such as detect an AR marker, or even perform visual odometry
- Just google "ROS package [whatever you want to do]" and you would be surprised how often something already exists to do it.

## OK, that makes some conceptual sense, but how does it work?

A lot of how ROS works can stay essentially black boxed, which is good because managing communication can get complicated very quickly. [look here](http://wiki.ros.org/ROS/Technical%20Overview) if you want more detail than you need to know about how the communication works.

### Topics
The first, and most common method of communication using ROS is publishing and subscribing to a ROS topic. You can think of the topic as a bulliten board: pieces of code or "nodes" can post or "publish" information to a topic, at which point it is available for any other node (or even you in the command line) to see. So node a could collect information and constantly post it to a topic, then node b can "subscribe" to the topic, at which point it can constantly see the most up to date information posted to that topic. In addition, node c, d, and e can also subscribe to the same topic, and they will also get the most up to date information. 

This is great for making code modular because as long as information is published to the topics a piece of code needs, it can run independently from the rest of the project. So you can debug a node by running the node and publishing test cases to a set of topics, then you can subscribe to the output topic terminal and check the results.

### Services
Another common method of communication is through a ros "Service". Rather than publishing information for anyone to see and having other nodes listen, ros services allow nodes to request information from another node. In this model, node a wants information from node b, so it sends a "service request" to node a, and node a replys with the information, or, node a may want node b to perfrom an action, so node a again sends a  service request, and node b performs the action, and should probably send back a boolean indicating the success of the action.

The first interaction I described is useful for requesting information which you don't want to collect continuously, while the second is a way to basically run a function in another node.

### ROS Master
In order to run any code which uses ROS, you also need to run a `ros master`. Usually this is done automatically through `ros launch` or by running `roscore`, but the ros master is what manages all of the communication between different nodes.