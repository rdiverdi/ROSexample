---
title: Tutorials
layout: template
filename: tutorials
--- 

# Tutorial Guide
This page is a guide through the ROS tutorials. It links out to each of the tutorials, but also includes a short comment about them to help you get through the tutorials efficiently.

Note: This guide links to all of the ROS *Indigo*

## contents
- [Initial Setup](#initial-setup-tutorials)
- [ROS Package Tutorials](#ros-package-tutorials)
- [Using ROS](#using-ros)
- [Custom Messages and Services](custom-messages-and-services)
- [Using ROS in Programs](using-ros-in-programs)
- [Built in Tools](built-in-tools)

## Initial Setup Tutorials

### [Install ROS (ROS Indigo on Ubuntu)](http://wiki.ros.org/indigo/Installation/Ubuntu)
This guide is pretty good, notes are:

- in the install step, use the "Desktop-Full" install
- in step 1.6 you can stop after running the commands in the first blue box

### [Configuring your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
This is a one-time setup to get a ROS filesystem setup on your computer.

- If you just installed ROS, skip to step 3
- Use the catkin instructions throughout the tutorials
- Technically, you can call you workspace anything you want and put it anywhere you want, but please call it `catkin_ws` and put it in your home directory: it will make it easier to work with everyone else in the class.

### [Navigating the ROS file system](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
Don't spend too much time on this tutorial, `roscd` is probably the only thing you will use regularly

- your "distro" is indigo

## ROS Package Tutorials

### [Creating a Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
This is listed next on the ROS website, but I think you should skip it and come back later: There is a lot you can learn about ROS that will be useful to know before you start playing around with making your own packages. I suggest skipping down to [Understanding ROS Nodes](#using-ros).

If you're about to go through this tutorial, you might want to stop now and ask, what even is a ROS package, and why do I want to make one?  A ROS package is a small group of related nodes (pieces of code), similar to a folder which contains a few related files. It is generally the smallest grouping of ROS code which is easily transferable between projects because a package can contain any of the main elements of a ROS project: it contains its own nodes, launch files, message definitions, and service definitions, meaning you can group almost any project into a single ROS package (that doesn't mean you need to keep everything in a single package, in fact, if you use existing code, you will use multiple packages in your project). 
So, the point of organizing your code into a package is that it gives you access to more of the ROS funcionality than just running a single piece of code, and it keeps your code organized and modular.

### [Building a ROS package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
This tutorial basically just tells you to go to your `catkin_ws` directory and run `catkin_make`, don't worry about the rest

## Using ROS

### [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
This tutorial is pretty good, it tells you how to start up ROS, run a node, and see what nodes are running. It should be pretty fast. This is also the beginning of the turtle-sim tutorials. In these tutorials you will start to control a turtle on your screen, which may seem very different from controlling a robot, but when you start working with a real robot, you will be able to control the robot in the same way.

### [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
This is the first tutorial where you really start to use the nice features of ROS. Don't worry too much about `rqt_graph`, it's just a quick way to visualize what is running, and how things are communicating. The `rostopic` tools this tutorial covers are extremely useful for debugging and testing code, and `rqt_plot` can also be useful, depending on what you want to see.

If you like videos, the video at the bottom of this tutorial covers everything from this tutorial and the previous one, but it's really long, definitely watch at double speed.

### [Understanding ROS services and parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
The tools in this tutorial are less useful than the last tutorial, but they can be really helpful if you know that they are there. It's also pretty fun because you get to see things changing again.

### [Using rqt_console and roslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
The second half of this tutorial (`roslaunch`) is far more useful than the first half. the first half talks about `rqt_console`, and before you used `rqt_graph`. All of the rqt tools, however, are easier to access through `rqt_gui` (run with `rosrun rqt_gui rqt_gui), which gives you access to all of the different rqt tools.

### [Using rosed to edit files in ROS](http://wiki.ros.org/ROS/Tutorials/UsingRosEd)
I honestly never use this, if you use vim or nano, you can look at it, otherwise skip it.

## Custom Messages and Services
This would be a good time to go back to the [package tutorials](#ros-package-tutorials) because you will need your code to be organized in a package in order to use custom messages and services.

### [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
This package discusses building your own ROS messages and services, I would suggest reading through it quickly, then looking more closely at it if you actually need it.

## Using ROS in Programs

### [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
Here's an example program which uses ROS (finally). Read through everything, figure out how it works, run the program (see next tutorial for how to run it), and play around. 

The repository associated with this website also has a hello world script along with two other example scripts which show how to use a class structure. (There will probably be a page explaining this better, and eventually this will link to that)

### [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
This just tells you how to run the code from the last tutorial

### [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
Same as publisher and subscriber but for services

### [Examining the Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)
Again, just runs the code from the previous tutorial

## Built in Tools

### [Recording and playing back data](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)
This talks about `rosbag`, which will be really useful, especially for working with a real robot. What rosbag lets you do is record everything that is posted to any (or a selection of) ros topics, so you can play it back later. The cool part is that, when you play it back, everything is published back to their topics, so you can run new code and have it react to the recorded input. This lets you adjust code and test it on real inputs without constantly running it on a physical robot.

### [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
This is supposed to be useful, but I don't actually use it that much, it's for debugging, and has an amusing name.

### Done for now
The links keep going, but this is basically the end of the basic ROS tutorials. We will also work with Rviz and TF, but those tutorials really don't belong in this section.