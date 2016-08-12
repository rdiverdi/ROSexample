#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
import numpy as np

ARRAY_SIZE=11

VEL = 0
TURN = 1

class Wpt():
	def __init__(self):
		rospy.init_node('wpt')

		self.array = np.ones([2, ARRAY_SIZE])
		self.array[VEL] = [-40, -30, -20, -10, -5, 10, 20, 30, 40, 50, 60]
		self.array[TURN]*=[0, 20, 60, 80, 95, 100, 95, 80, 60, 20, 0]
		for i in self.array:
			for j in i:
				j = int(j)

		self.obst_pub = rospy.Publisher('/wpt/cmd_vel', Int8MultiArray, queue_size=1)
		
		self.msg = Int8MultiArray()

	def test(self):
		r = rospy.Rate(2)
		while not rospy.is_shutdown():
			self.msg.data = self.array.reshape([1, 2*ARRAY_SIZE]).astype(int).tolist()[0]
			self.obst_pub.publish(self.msg)
			r.sleep()

if __name__ == "__main__":
	test = Wpt()
	test.test()