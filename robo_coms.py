#!/usr/bin/env python

""" Base structure for communicating with ROS """
""" ## = required for all programs using ROS
     # = specific to this example """

import rospy ##import ros
import serial
import json
from geometry_msgs.msg import Twist #import the ros messages we want to use
from std_msgs.msg import Int16, Int16MultiArray

class ArduinoComs(object): #classes make things better, promise
    """ This class encompasses the entire node """
    def __init__(self):
        ''' setup ROS stuff '''
        rospy.init_node('arduino_communicator') ## initialize node

        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        rospy.Subscriber('blink_rate', Int16, self.blink_cb)
        self.ir_pub = rospy.Publisher('ir_sensors', Int16MultiArray, queue_size=1)
        self.ser = serial.Serial('/dev/ttyAMA0', 115200)

    def cmd_vel_cb(self, msg):
        vel = str(msg.linear.x)
        turn = str(msg.angular.z)
        self.ser.write('{"type":1, "vel":'+vel+', "turn":'+turn+'}')
    def blink_cb(self, msg):
        rate = str(msg.data)
        self.ser.write('{"type":2, "rate":'+rate+'}')

    def run(self):
        """ main run loop """
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
          if self.ser.inWaiting():
            self.ser.readline()
            msg = self.ser.readline()
            try:
              received = json.loads(msg)
              msg_received = True
            except:
              msg_received = False
            if msg_received:
              if received['type'] == 3:
                msg = Int16MultiArray()
                msg.data = received['ir']
                self.ir_pub.publish(msg)
            self.ser.flushInput()
          r.sleep()

if __name__ == '__main__':
    "run above code"
    node = ArduinoComs()
    node.run()
