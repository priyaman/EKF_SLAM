#!/usr/bin/python
import rospy
import numpy as np
import rosbag
import sys
from tf.transformations import euler_from_quaternion
from math import sqrt
import math
import matplotlib.pyplot as plt
import matplotlib.colors as colorlib
import itertools
import tf

#CONSTANTS


#CLASSES
class Tag_position:
	x = 0.0
	y = 0.0
	z = 0.0
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z
	def __str__(self):
		return "(" + str(self.x) + "," + str(self.y) + "," + str(self.z) + ")"
	pass


#HELPER METHODS

#Assuming tag pose wrt robot pose
def get_tag_range(pose_tag):
	return pose_tag.position.x**2 + pose_tag.position.y**2 + pose_tag.position.z**2

#Assuming robot thetha is 0 and returning only 'z' component
def get_tag_bearing(pose_tag):
	return euler_from_quaternion(pose_tag.orientation)[2]
	

#to,from
def get_required_rotation(line_angle,pose_theta):
	if(pose_theta - line_angle > 0 and pose_theta - line_angle < math.pi ):
		 return -1*(math.pi - abs(abs(line_angle - pose_theta) - math.pi)) 
	else:
		 return (math.pi - abs(abs(line_angle - pose_theta) - math.pi))

def send_robot_pose(msg):
	br = tf.TransformBroadcaster()
	#print "Robot Orientation:" + str(euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2])#str(msg.pose.pose.orientation.x) + "," + str(msg.pose.pose.orientation.y)
	br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0), (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), rospy.Time.now(), "robot", "world")

def draw_trajectory():
	bag = rosbag.Bag('velocitybag.bag')
	xlist = []
	ylist = []
	tag_dict_x = {}
	tag_dict_y = {}
	robot_x = 0.0
	robot_y = 0.0
	robot_z = 0.0
	for topic, msg,t  in bag.read_messages(topics=['/odom','/tag_detections']):
		#print topic
		#print t
		#raw_input()
		if(topic =='/odom'):
			robot_x = msg.pose.pose.position.x		
			robot_y = msg.pose.pose.position.y
			robot_z = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
			#Send robot pose transform			
			send_robot_pose(msg)

			xlist.append(robot_x)
			ylist.append(robot_y)
		if(topic == '/tag_detections' and len(msg.detections)>0):
			for tag_itr in range(0,len(msg.detections)):
				#raw_input()
				curr_x_list = []
				curr_y_list = []
				if msg.detections[tag_itr].id in tag_dict_x:
					curr_x_list = tag_dict_x[msg.detections[tag_itr].id]	

				if msg.detections[tag_itr].id in tag_dict_y:
					curr_y_list = tag_dict_y[msg.detections[tag_itr].id]
					
				#range_to_landmark = math.sqrt(msg.detections[tag_itr].pose.pose.position.x**2 + msg.detections[tag_itr].pose.pose.position.x**2 + msg.detections[tag_itr].pose.pose.position.x**2)
				#bearing_to_landmark = get_required_rotation(math.atan2(msg.detections[tag_itr].pose.pose.position.y,msg.detections[tag_itr].pose.pose.position.x),0)
				#BACKUP
				#print str(tag_itr) + ":" + str(bearing_to_landmark)
				R = np.matrix([[math.cos(robot_z), -math.sin(robot_z)], [math.sin(robot_z), math.cos(robot_z)]])
				#print R
				tag_reading = np.matrix([msg.detections[tag_itr].pose.pose.position.z, msg.detections[tag_itr].pose.pose.position.x ])
				#print tag_reading				
				tag_reading = tag_reading * R
				curr_x_list.append(robot_x + tag_reading.item(0))
				curr_y_list.append(robot_y - tag_reading.item(1))
				#print "Tag pose for id=" + str(msg.detections[tag_itr].id) + " is " + str(msg.detections[tag_itr].pose.pose.position)
				
				tag_dict_x[msg.detections[tag_itr].id] = curr_x_list
				tag_dict_y[msg.detections[tag_itr].id] = curr_y_list
	

	plt.scatter(xlist, ylist)	
	max_tag_id = max(tag_dict_x.keys(), key = int)
	colors = itertools.cycle(colorlib.cnames)
	for tag_id in tag_dict_x.keys():
		curr_x_list = tag_dict_x[tag_id]
		curr_y_list = tag_dict_y[tag_id]
		plt.scatter(curr_x_list, curr_y_list, c=next(colors))
	 
		
	plt.show(block = "True")
	bag.close()

if __name__ == '__main__':
	rospy.init_node('robot')
	draw_trajectory()	
	#rospy.Subscriber('robot', turtulesim.msg.pose, send_robot_pose, "robot")
	rospy.spin()

