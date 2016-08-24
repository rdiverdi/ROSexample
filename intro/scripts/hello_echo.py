#!/usr/bin/env python
"""
This is an example of python code which uses ROS to repeat messages from one topic
on a second topic.
It is useful for testing two way communication with a device because it both receives and
sends messages.
This node also reports its own status, allowing you to test it sending messages separately
from receiving them
"""
import rospy
from std_msgs.msg import String, Bool # I want to use string and boolean messages

class HelloEcho():
    def __init__(self):
          """ initialize the ROS node and setup publishers and subscribers """
          rospy.init_node("hello_echo") #this node is named 'hello_echo'
          self.echo_pub = rospy.Publisher("/echo_world", String, queue_size=10)
            #it publishes a string to the topic 'echo_world'
          self.status_pub = rospy.Publisher("/echo_status", Bool, queue_size=10)
            #it publishes a boolean to 'echo_status'
          hello_sub = rospy.Subscriber("/hello_world", String, self.hello_cb)
            #it subscribes to 'hello_world' and runs 'self.hello_cb' when it receives a message

    def hello_cb(self, msg):
        """ runs when a message is received on topic 'hello_world'
        re-publishes the received message on 'echo_world'"""
        recieved = msg.data #get the received string
        self.echo_pub.publish(recieved) #publish the received string to 'echo_world'

    def run(self):
        """ main run loop for HelloEcho
        Runs at 2 Hz and pubhises a status message """
        r = rospy.Rate(2) #set rate (in Hz)
        while not rospy.is_shutdown(): #run until ROS is shutdown
            self.status_pub.publish(True) # publish True to the status topic
            r.sleep() #wait until next time loop should run

if __name__ == "__main__":
    main = HelloEcho()
    main.run()

