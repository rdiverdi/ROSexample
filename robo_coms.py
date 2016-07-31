#!/usr/bin/env python

""" Base structure for communicating with ROS """
""" ## = required for all programs using ROS
     # = specific to this example """

import rospy ##import ros
import serial
from geometry_msgs.msg import Twist #import the ros messages we want to use
from std_msgs.msg import Int16

class ArduinoComs(object): #classes make things better, promise
    """ This class encompasses the entire node """
    def __init__(self):
        ''' setup ROS stuff '''
        rospy.init_node('arduino_communicator') ## initialize node

        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        rospy.Subscriber('blink_rate', Int16, self.blink_cb)
        self.ser = serial.Serial('/dev/ttyUSB0')

    def cmd_vel_cb(self, msg):
        vel = str(msg.linear.x)
        turn = str(msg.angular.z)
        self.ser.write('{"type":1, "vel":'+vel+', "turn":'+turn+'}')
    def blink_cb(self, msg):
        rate = str(msg.data)
        self.ser.write('{"type":2, "rate":'+rate+'}')

    def run(self):
        """ main run loop """
        rospy.spin()

if __name__ == '__main__':
    "run above code"
    node = ArduinoComs()
    node.run()
