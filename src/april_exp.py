#!/usr/bin/env python

import rosbag
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

bag = rosbag.Bag('velocitybag.bag')
id_to_draw = 6
x_list = []
y_list = []
robot_x = 0.0
robot_y = 0.0
br_count=0
start = False
for topic, msg,t  in bag.read_messages(topics=['/tag_detections','/odom']):
#/cmd_vel_mux/input/teleop,'/mobile_base/events/bumper','/tag_detections', '/odom'
#xy, xz, yx, yz, zx, zy
	if topic == '/tag_detections':
		for i in range(0, len(msg.detections)):
			if(msg.detections[i].id == id_to_draw):
				print "drawing"
				x_list.append(robot_x + msg.detections[i].pose.pose.position.z)
				y_list.append(robot_x - msg.detections[i].pose.pose.position.x)
				start = True			
			else:
				if start:
					br_count+=1
		if br_count >15 and start:
			break
	if topic == '/odom':
		robot_x = msg.pose.pose.position.x		
		robot_y = msg.pose.pose.position.y
		
plt.scatter(x_list, y_list)
plt.show(block='False')
		
		

print "-----------------------------------------"
			
bag.close()
