---
title: Tutorials
layout: template
filename: tutorials
--- 

# Tutorial Guide
This page is a guide through the ROS tutorials. It links out to each of the tutorials, but also includes a short comment about them to help you get through the tutorials efficiently. The title links go to the c++ versions of the tutorials, but I've also included links to the python versions at the bottom of the relavent tutorials.

Note: This guide links to ROS *Indigo* tutorials

## contents
- [Initial Setup](#initial-setup-tutorials)
- [ROS Package Tutorials](#ros-package-tutorials)
- [Using ROS](#using-ros)
- [Custom Messages and Services](#custom-messages-and-services)
- [Using ROS in Programs](#using-ros-in-programs)
- [Built in Tools](#built-in-tools)
- [Other Tutorials](#other-tutorials)

These tutorials can often get long and tedious; however, most of the tools are extremely helpful when you start writing and debugging code. The first section which focuses on writing code is the [Using ROS in Programs section](using-ros-in-programs). Hopefully my comments will help you skip over the less useful parts and get through these quickly. Feel free to take these out of order, I tried to group the tutorials to make it easier to jump around in a way that makes sense.

## Initial Setup Tutorials

### [Install ROS (ROS Indigo on Ubuntu)](http://wiki.ros.org/indigo/Installation/Ubuntu)
This guide is pretty good, notes are:

- in the install step, use the "Desktop-Full" install
- in step 1.6 you can stop after running the commands in the first blue box

### [Configuring your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
This is a one-time setup to get a ROS filesystem setup on your computer.

- If you just installed ROS, skip to step 3
- Use the catkin instructions throughout the tutorials
- Technically, you can call your workspace anything you want and put it anywhere you want, but please call it `catkin_ws` and put it in your home directory: it will make it easier to work with everyone else.

### [Navigating the ROS file system](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
Don't spend too much time on this tutorial, `roscd` is probably the only thing you will use regularly

- your "distro" is indigo

## ROS Package Tutorials

I listed these tutorials in the same order as on the ROS website, but the next two tutorials feel out of place here. If you are running through tutorials to learn ROS, I suggest skipping down to the [Using ROS section](#using-ros) below and coming back here once you have a better idea of how ROS manages code.

### [Creating a Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

If you're about to go through this tutorial, you might want to stop now and ask, what even is a ROS package, and why do I want to make one?  A ROS package is a small group of related nodes (pieces of code), similar to a folder which contains a few related files. It is generally the smallest grouping of ROS code which is easily transferable between projects because a package can contain any of the main elements of a ROS project: it contains its own nodes, launch files, message definitions, and service definitions, meaning you can group almost any project into a single ROS package (that doesn't mean you need to keep everything in a single package, in fact, if you use existing code, you will use multiple packages in your project). 
So, the point of organizing your code into a package is that it gives you access to more of the ROS funcionality than just running a single piece of code, and it keeps your code organized and modular.

### [Building a ROS package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
This tutorial basically just tells you to go to your `catkin_ws` directory and run `catkin_make`, don't worry about the rest

## Using ROS

### [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
This tutorial is pretty good, it tells you how to start up ROS, run a node, and see what nodes are running. It should be pretty fast. This is also the beginning of the turtle-sim tutorials. In these tutorials you will start to control a turtle on your screen, which may seem very different from controlling a robot, but when you start working with a real robot, you will be able to control the robot in the same way.

### [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
This is the first tutorial where you really start to use the nice features of ROS. Don't worry too much about `rqt_graph`, it's just a quick way to visualize what is running, and how things are communicating. The `rostopic` tools this tutorial covers are extremely useful for debugging and testing code, and `rqt_plot` can also be useful, depending on what you want to see. ps. tab complete is your friend when using `rostopic pub`: it will automatically give you a template for your message.

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

The repository associated with this website also has a hello world script along with two other example scripts which show how to use a class structure. The [this repo](this_repo) page has more information on those scripts.

[python version](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

### [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
This just tells you how to run the code from the last tutorial

### [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
Same as publisher and subscriber but for services. I use ROS services much less than publishers and subscribers, but it's useful to know that the option is there.

[python version](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

### [Examining the Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)
Again, just runs the code from the previous tutorial

## Built in Tools

### [Recording and playing back data](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)
This talks about `rosbag`, which will be really useful, especially for working with a real robot. What rosbag lets you do is record everything that is posted to any (or a selection of) ros topics, so you can play it back later. The cool part is that, when you play it back, everything is published back to their topics, so you can run new code and have it react to the recorded input. This lets you adjust code and test it on real inputs without constantly running it on a physical robot.

### [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
This is supposed to be useful, but I don't actually use it that much, it's for debugging, and has an amusing name.

### Other Tutorials
The links keep going, but this is basically the end of the basic ROS tutorials. We will also work with Rviz and TF, but those tutorials really don't belong in this section. There are a few more things which I think are useful, so I'm going to put links here for reference.

### [roslaunch Files](http://wiki.ros.org/roslaunch/XML)
Once you have a lot of nodes running in a project, you will want to use a launch file to run all of them at once. This was discussed in [Using rqt_console and roslaunch]((http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch), but this is a much more complete list of the options

### [Using ROS Parameters](http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters)
One of the earlier tutorials discussed ROS parameters, but they never showed you how to use them in a program, so I thought this would be useful.

[python version](http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters)

### [Running ROS across multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
ROS makes communication between computers extremely simple, but there is a little inital setup to make eveerything work. I use this alot, especially for seeing what is happening on a robot with its own roscore from my computer.
