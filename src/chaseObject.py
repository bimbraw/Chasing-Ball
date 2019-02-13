#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Point, Twist
from math import pi
import numpy as np
import cv2

class chaseObject:
	def __init__(self):
		rospy.init_node('chaseObject', anonymous = True)
		self.target_dist = .3
		
		#Values obtained by experimentation		
		self.Kp_angle = 1;
		self.Kp_dist = 100;

		#subscribing to objectPoint and publishing /cmd_vel 
		self.polar_sub = rospy.Subscriber("point", Point, self.callback)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

	def callback(self, data):
		
		#defining errors
		angle_error = data.x
		dist_error = data.y - self.target_dist
		
		#defining Twist()
		msg = Twist()
		msg.angular.z = angle_error*self.Kp_angle

		#61.1 degrees correspond to 0.543 radians
		if abs(angle_error) > 0.543:
			angle_error = 0
		if abs(msg.angular.z) > 0.3:
			msg.angular.z = 0.3*np.sign(msg.angular.z)
		#bsaically if angle error is less than half the fov in x direction		
		if angle_error < 31.1*pi/180:
			msg.linear.x = dist_error*self.Kp_dist
			if abs(msg.linear.x) > 10:
				msg.linear.x = 0.03*np.sign(msg.linear.x)	
			else:
				msg.linear.x = 0
				msg.angular.z = 0

		#publishing velocity messages
		self.vel_pub.publish(msg)

if __name__ == '__main__':
	chaseObject = chaseObject();
	rospy.spin()
