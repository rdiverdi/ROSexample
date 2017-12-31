#!/usr/bin/env python
# the top line indicates that this program should be run using python
## it is required for all python scripts you will run using ROS

import rospy #imports the basic ros libraries for python
from std_msgs.msg import String #imports the string message type used by ROS

def talker():
    pub = rospy.Publisher('hello_world', String, queue_size=10)
        #sets up a publisher on the topic "hello_world" which will publish a String
    rospy.init_node('talker')
        #initializes a ros node with the name "talker"
    rate = rospy.Rate(4) # 10hz
        #setup the rate at which to run the following loop (in hz)
    while not rospy.is_shutdown(): #as long as ROS is running
        hello_str = "hello world %s" % round(rospy.get_time(), 2)
            #string containing "hello world" and the current time
        rospy.loginfo(hello_str) #write string to a log
        pub.publish(hello_str) #publish the string
        rate.sleep()
            #wait until the next time this loop should run according to rate

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

