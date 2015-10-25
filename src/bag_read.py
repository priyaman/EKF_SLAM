#!/usr/bin/env python

import rosbag
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import numpy as np

class Custom_Point(Point):
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

	def __add__(self, other):
		total_x = self.x + other.x
		total_y = self.y + other.y
		total_z = self.z + other.z
		return Custom_Point(total_x, total_y, total_z)
	
	def __div__(self, other):
		total_x = self.x/other
		total_y = self.y/other
		total_z = self.z/other
		return Custom_Point(total_x, total_y, total_z)

	def __truediv__(self, other):
		total_x = self.x/other
		total_y = self.y/other
		total_z = self.z/other
		return Custom_Point(total_x, total_y, total_z)

	pass

class Pose_v2(Pose):
	def __init__(self, pose_x, pose_y, pose_z, orientation_x, orientation_y, orientation_z, orientation_w):
		super(Pose_v2, self).__init__()
		self.position.x = pose_x
		self.position.y = pose_y
		self.position.z = pose_z
		self.orientation.x = orientation_x
		self.orientation.y = orientation_y
		self.orientation.z = orientation_z
		self.orientation.w = orientation_w

	def __add__(self, other):
		total_pos_x = self.position.x + other.position.x
		total_pos_y = self.position.y + other.position.y
		total_pos_z = self.position.z + other.position.z
		total_ori_x = self.orientation.x + other.orientation.x
		total_ori_y = self.orientation.y + other.orientation.y
		total_ori_z = self.orientation.z + other.orientation.z
		total_ori_w = self.orientation.w + other.orientation.w
		return Pose_v2(total_pos_x, total_pos_y, total_pos_z, total_ori_x, total_ori_y, total_ori_z, total_ori_w)

	def __div__(self, other):
		self.position.x = self.position.x/other
		self.position.y = self.position.y/other
		self.position.z = self.position.z/other
		self.orientation.x = self.orientation.x/other
		self.orientation.y = self.orientation.y/other
		self.orientation.z = self.orientation.z/other
		self.orientation.w = self.orientation.w/other
		return self

	def __truediv__(self, other):
		self.position.x = self.position.x/other
		self.position.y = self.position.y/other
		self.position.z = self.position.z/other
		self.orientation.x = self.orientation.x/other
		self.orientation.y = self.orientation.y/other
		self.orientation.z = self.orientation.z/other
		self.orientation.w = self.orientation.w/other
		return self

	pass
		

bag = rosbag.Bag('velocitybag.bag')
average_odom = Odometry()
odom_list = []
count = 0
for topic, msg,t  in bag.read_messages(topics=['/cmd_vel_mux/input/teleop']):
#for msg  in bag.read_messages(topics=['/odom']):
#/cmd_vel_mux/input/teleop,'/mobile_base/events/bumper','/tag_detections', '/odom'
	print str(t) + ":" + str(msg)
	#odom_list.append(Custom_Point(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z))
	#odom_list.append(Pose_v2(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
	#print "-----------------------------------------"

bag.close()

           
