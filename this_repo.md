---
title: This Repo
layout: template
filename: this_repo
--- 

# What is in this repository

This repository contains code for the Olin College, Fundamentals of Robotics class, but the intro folder contains some simple examples of ROS code. To use code in this repository, fork the repository and clone it into the `src` folder of your catkin workspace.

## Intro Folder
The intro folder is a ROS package demonsrating a few different things you can do with ROS. All of the examples are duplicated in python (scripts folder) and c++ (src folder). Before looking through all of the scripts in this folder, if you are looking for exercices to help you learn ROS, I reccomend reading though the descriptions in this section (stopping at the heading "More aobut the code"), and writing your own versions of each of the scripts. (If you subscribe and publish to the same topics, you should be able to use your script as a drop in replacement for the scripts in this folder). You can use the included scripts as an answer key, but there are a number of extra features in the included code which are discusssed in the "More about the code" section.

### `src` Folder
This folder contains three scripts which publish and subscirbe to a few topics:

- `hello_world.cpp` is a copy of the hello world script from the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29), publishing a hello world message to the topic `/hello_world`. To run this script, use `rosrun intro talker`.
- `hello_echo.cpp` demonstrates a subscriber and publisher in the same script. It listens to the `/hello_world` topic and echos the messages it hears to the `/echo_world` topic. Run this code using `rosrun intro echo`. This node also publishes a status message to `/echo_status` every two seconds, so you can check that the code is running without publishing anything to the `/hello_world` topic. In this code, the second message is published from inside the callback function from the first message. 
- `hello_counter.cpp` is a little different from `hello_echo.cpp` because it publishes a message from the main run loop, and the callback function just updates a variable. This script subscribes to `/hello_world` and counts the number of messages it has received. It then periodically publishes a count of how many messages it has received to `/message_count`. run this using `rosrun intro counter`.

These three pieces of code can be run together to move information around: if you run all three at once you will see hello world messages on the `/hello_world` and `/hello_echo` topics, as well as a count of the number of messages published to `/hello_world` on the `/message_count` topic. However, you can also see how ROS's modularity can make it easy to test individual pieces of this code. You can run the echo node by itself and publish messages to `/hello_world` from the terminal, then those messages will be echoed on the topic `/echo_world`, or you can run the counter node by itself, or with the echo node, and you, again, can publish messages to `/hello_world`, and test just the counter node.

### `scripts` Folder
This folder contains scripts with the same functionality as the src folder, except implemented in python.

- `hello_world.py`: publishes to `/hello_world`. run with `rosrun intro hello_world.py`
- `hello_echo.py`: echos messages from `/hello_world` to `/echo_world`. run with `rosrun intro hello_echo.py`
- `hello_counter.py`: counts messages on `/hello_world` and publishes count to `/message_count`. run with `rosrun intro hello_counter.py`

### `launch` Folder
This contains two very simple examples of launch files, one for the c++ implementatoins and one for the python implementations. In each case, the launch file runs all three scripts described above. Note: you don't need to run roscore before using a roslaunch, roscore will be run automatically if it is not already running.

## More about the code

### Code Structure
The structure of ROS code is esentially the same in both python and C++, the differences are mostly in syntax, and are documented better in the ROS tutorials. Looking at the code in this repository, the hello world scripts are essentially the simplest programs which can interface with ROS. They intialize a ROS node, setup a publisher, and publish a message in a main run loop.

While this is good for seeing the base requirements for a ROS program, it is not how I would suggest organizing your code. In the `hello_echo` scripts, the code is organized into a class structure. The class structure works well with ROS code because it organizes the code into separate initialization and run sections. You can use the initialization section to initialize the node and setup the publishers and subscribers. Using a class structure also allows you to define callback functions in a different section of code than where you declare the subscribers (this makes longer ROS programs much easier to read). Overall, organizing a ROS node into a class makes the code better organized and more readable.

Some other notable differences between `hello_echo` and `hello_counter` are the timing behaviors. In `hello_echo`, there is a publisher inside the callback function, so the message is echoed as soon as it is received (in c++ this only works if you're using `ros::spin()`, which is why the script uses the timer function to publish to `/echo_status`). In `hello_counter`, the callback function just sets a class variable, then the current count is published at a regular interval set by the counter script. Both options can be useful, but in general you want to make sure callback functions are short and run quickly, so setting an internal variable is usually the prefered method. Thinking about timing is always important, especially considering what happens if you miss mssages, fall behind, or don't get a message as fast as you expected. For example, if `hello_counter` doesn't recieve a new message within 2 seconds, it publishes the same count twice. If `hello_echo` ran the same way, it would look like it had recieved a second coppy of a message when that message didn't actually exist. Similarly, if it recieved two messages within 2 seconds, it would only publish the later, and the first one would be lost.

### ROSlaunch features
If you run the three scripts separately then run `rqt_graph`, you will see that both the echoer and the counter are subscribed to `hello_world`. If you then kill those and instead run one of the lauch scripts (and refresh the rqt graph), you will see a couple of differences.

First, the counter is now subscribed to the output of the echoer. In the launch script, this is done with the line `<remap from="hello_world" to="echo_world"/>`. This is within the tag for the counter node, and dictates that, within the counter node, all references to the `hello_world` topic with the `echo_world` topic. This remapping can also be done as a command line argument when running the counter node by running it as `rosrun intro counter hello_world:=echo_world`.

The second difference is the topic names are now preceded by `cpp/` or `python/` (depending on which launch file you ran), and the three nodes are encased in a box labeled either `cpp` or `python`. This indicates that the three nodes are running in a namespace. The launch file dictates this namespace using the `<group>` tag. The second line in the launch file (`<group ns="cpp"`) establishes a group of nodes, all of which will be in the `cpp` namespace. The `</group>` line ends this group: any nodes established after this line would not be in the namespace. Any other launch file arguments defined within a group are contained to that group. Namespaces are good for grouping nodes or distinguishing between multiple instances of the same node. The namespace of a terminal can be set by running `export ROS_NAMESPACE=<namespace>`.

Finally, if you `rostopic echo /echo_world`, you'll notice that the message now starts wtih "launch" rather than "echo". That is because the launch file sets a ROS parameter. The line `<param name="prefix" value="launch"/>` sets the prefix parameter to launch (its default parameter is "echo"). This parameter can be set in the command line as well by adding `_prefix:=<value>` to the command when running the echoer with rosrun. Parameters just become variables in the code, and they can be strings, integers, or booleans, which makes them extremely flexible.

### Global Namespace
After running one of the lauch files, open a terminal and run `rostopic echo /echo_status`. This is a topic published by the echoer to indicate that it is running. Now, if you run `rqt_graph`, you will see the namespace box, as before, but now there's a new node outside of the namespace (this is your terminal) listening to the topic `/echo_status`, which also has no reference to the namespace. If you open the `hello_echo` script, you will see that one publisher is setup to publish to the `"echo_world"` topic, while the other publishes to `"/echo_status"`. The `/` in front of `"echo_status"` indicates that the `"echo_status"` topic is in the "global" topic namespace, therefore it is uneffected by the namespce dictated in the launch file.

### Parameters
In the python script `hello_echo.py`, the line `self.prefix = rospy.get_param("~prefix", "echo")` gets the ros parameter `prefix` if it exists, otherwise it uses the default value `"echo"`. ROS parameters can exist as global parameters (ex. `"/prefix"`), within a namespace (ex. `"/namespace/prefix"`), or within a node (ex. `"hello_echo/prefix"`). We are using the later, and (in python), `"~"` is shorthand for the node's private namespace. In C++, the equivalent line is `nh.param<std::string>("prefix", prefix, "echo")`. This is essentially the same as the python line, except the variable where the value is stored is an input, however, in order to use the local namespace, we need the line `ros::NodeHandle nh("~")` (which is directly above the param line). This second nodehandle gives us easy access to private parameters.

### Shutdown Handling
In python, there is a very easy way to run a bit of code as the node is shut down. If you run the python launch script, echo the `/echo_status` topic, and then shut down the launch script, you will see a `False` published as the node shuts down. In the file, there is a method named `on_kill` which is referenced in the last line of the `__init__` method: `rospy.on_shutdown(self.on_kill)`. This line indicates that, when the node is killed, it shoud call the `on_kill` method before shutting down. This is extremely useful for things like triggering an e-stop when you shut down a node that is controlling a robot. There are a couple of ways to do this in c++, but they aren't as straight forward and have tradeoffs depending on what you're doing, so I haven't included them.
