#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from math import pi
import numpy as np
import cv2

class getObjectRange:
	def __init__(self):
		self.scan = LaserScan()
		self.fov_h = 62.2
		rospy.init_node('getObjectRange', anonymous = True)
		
		#subscribing to image location and Lidar messages and publishing values to chaseObject
		self.pt_sub = rospy.Subscriber("imageLocation", Point, self.point_callback)
		self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		self.polar_pub = rospy.Publisher("point", Point, queue_size = 1)

	def point_callback(self,data):
		#I used the camerav2_320X240_5fps.launch
		x = data.x * self.fov_h/320
		x = -x + self.fov_h/2
		z = data.z * self.fov_h/320

		#converting all values to radians
		x = x * pi/180
		r = z * pi/180

		#just for debugging
		print(x, r)
		
		#defining initial and final bounds
		i = round((x - r - self.scan.angle_min)/self.scan.angle_increment)
		f = round((x + r - self.scan.angle_min)/self.scan.angle_increment)
		
		#reading LIDAR values
		ranges = np.array(self.scan.ranges)
		if i < 0 and f > 0:
			object_ranges = np.concatenate((ranges[359-i:], ranges[0:f]))
		else:
			object_ranges = ranges[i:f]
		
		#object range
		print(object_ranges)
		
		#creating a mask to eliminate values which are not required
		mask = np.bitwise_and(object_ranges > self.scan.range_min, object_ranges < self.scan.range_max)
		if object_ranges[mask].size > 0:
			min_dist = object_ranges[mask].min()
		else:
			min_dist = 0.3
		msg = Point()
		msg.x = x
		msg.y = min_dist
		
		#printing minimum distance
		print(min_dist)

		#publishing the values
		self.polar_pub.publish(msg)

	def scan_callback(self,data):
		self.scan = data

if __name__ == '__main__':
	getObjectRange = getObjectRange();
	rospy.spin()
