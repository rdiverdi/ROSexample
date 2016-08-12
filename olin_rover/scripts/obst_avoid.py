#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8MultiArray, Int16MultiArray
import numpy as np

ARRAY_SIZE = 11
NUM_SENSORS = 1

class Obst_Avoid():
	def __init__(self):
		rospy.init_node('obst_avoid')

		self.ir_readings = [0]*NUM_SENSORS
		self.obst_array = [-3, -7, -10, -7.5, -3.5]
		self.speed_array = [1, 1, 1, 1, 1, 0, -1, -2.4, -4, -6, -10]
		self.obst = 3

		self.array = np.zeros([2, ARRAY_SIZE])

		rospy.Subscriber('/ir_sensors', Int16MultiArray, self.ir_sensor_cb)
		self.obst_pub = rospy.Publisher('/obst/cmd_vel', Int8MultiArray, queue_size=1)
		
		self.msg = Int8MultiArray()

	def ir_sensor_cb(self, msg):
		self.obst = (msg.data[0] - 100)/40. #scale from 100:500 to 0:10
		if self.obst > 10:
			self.obst = 10
		elif self.obst < 0:
			self.obst = 0

	def test(self):
		r = rospy.Rate(2)
		while not rospy.is_shutdown():
			self.array[0] = [i*self.obst**2/10. for i in self.speed_array]
			self.array[1][5-2:5+3] = [i*self.obst**2/10. for i in self.obst_array]
			self.msg.data = self.array.reshape([1, 2*ARRAY_SIZE]).astype(int).tolist()[0]
			self.obst_pub.publish(self.msg)
			r.sleep()

if __name__ == "__main__":
	test = Obst_Avoid()
	test.test()