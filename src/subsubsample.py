#!/usr/bin/python
import rospy
import numpy as np
import rosbag

#EXTEND CLASS TO SUPPORT ADD
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from hw_2.msg import AprilTagDetection
from hw_2.msg import AprilTagDetectionsArray
from hw_2.msg import EKFSlamInput
import numpy as np

	
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

class Vector3_v2(Vector3):
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
	def __add__(self, other):
		self.x = self.x + other.x
		self.y = self.y + other.y
		self.z = self.z + other.z
		return self
	def __div__(self, other):
		self.x = self.x / other
		self.y = self.y / other
		self.z = self.z / other
		return self
	pass


class Twist_v2(Twist):
	def __init__(self, linear, angular):
		super(Twist_v2, self).__init__()
		self.linear = linear
		self.angular = angular

	def __add__(self, other):
		self.linear.x = self.linear.x + other.linear.x
		self.linear.y = self.linear.y + other.linear.y
		self.linear.z = self.linear.z + other.linear.z
		self.angular.x = self.angular.x + other.angular.x
		self.angular.y = self.angular.x + other.angular.y
		self.angular.z = self.angular.x + other.angular.z
		return self

	def __div__(self, other):
		self.linear.x = self.linear.x / other
		self.linear.y = self.linear.y / other
		self.linear.z = self.linear.z / other
		self.angular.x = self.angular.x / other
		self.angular.y = self.angular.y / other
		self.angular.z = self.angular.z / other
		return self

	def __truediv__(self, other):
		self.linear.x = self.linear.x / other
		self.linear.y = self.linear.y / other
		self.linear.z = self.linear.z / other
		self.angular.x = self.angular.x / other
		self.angular.y = self.angular.y / other
		self.angular.z = self.angular.z / other
		return self
	pass

		

bag = rosbag.Bag('subsampled_velocitybag.bag')
bag_end = rospy.Time(bag.get_end_time())
start_time = rospy.Time(bag.get_start_time())
#Data averaging Time
data_averaging_time = 1
end_time = start_time + rospy.Duration(data_averaging_time)

with rosbag.Bag('subsubsampled_velocitybag.bag', 'w') as outbag:
	while end_time < bag_end:
		average_odom_list = []
		average_t_list = []
		average_input_list = []
		average_tag_list = []
		apriltags_dict = {}
		for topic, msg,t  in bag.read_messages(topics=['/cmd_vel_mux/input/teleop','/mobile_base/events/bumper','/tag_detections', '/odom'], start_time=start_time, end_time=end_time):
			#print str(t) + ":" + str(topic)
			#Append time for averaging
			average_t_list.append(t.to_sec())
			#Append odom for averaging
			if topic == '/odom':
				average_odom_list.append(Pose_v2(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))
			#Append teleop input for averaging
			elif topic == '/cmd_vel_mux/input/teleop':
				average_input_list.append(Twist_v2(msg.linear, msg.angular))
			#Append tag_data for averaging (group by tag_id)
			elif topic == '/tag_detections':
				for tag_itr in range(len(msg.detections)):
					if msg.detections[tag_itr].id not in apriltags_dict:
						apriltags_dict[msg.detections[tag_itr].id] = [Pose_v2(msg.detections[tag_itr].pose.pose.position.x,msg.detections[tag_itr].pose.pose.position.y,msg.detections[tag_itr].pose.pose.position.z,msg.detections[tag_itr].pose.pose.orientation.x,msg.detections[tag_itr].pose.pose.orientation.y,msg.detections[tag_itr].pose.pose.orientation.z,msg.detections[tag_itr].pose.pose.orientation.w)]
					else:
						apriltags_dict[msg.detections[tag_itr].id].append(Pose_v2(msg.detections[tag_itr].pose.pose.position.x,msg.detections[tag_itr].pose.pose.position.y,msg.detections[tag_itr].pose.pose.position.z,msg.detections[tag_itr].pose.pose.orientation.x,msg.detections[tag_itr].pose.pose.orientation.y,msg.detections[tag_itr].pose.pose.orientation.z,msg.detections[tag_itr].pose.pose.orientation.w))
			#must be bumper event (not caring about timestamp)			
			else: 
				outbag.write(topic, msg, t)

		#Calculate average time
		timestamp = np.mean(average_t_list)
		#Calculate average odom
		average_odom = np.mean(average_odom_list)
		#Calculate teleop input
		average_input = np.mean(average_input_list)
		#Calculate tag distance
		detections = []
		for tag_id, tag_data_list in apriltags_dict.items():
			average_tag_pose = np.mean(tag_data_list)
			average_tag_data = AprilTagDetection()
			average_tag_data.id = tag_id
			average_tag_data.pose.pose = average_tag_pose
			detections.append(average_tag_data)
	  	detections_msg = AprilTagDetectionsArray()
		detections_msg.detections = detections

		#Create EKF Slam Message
		ekf_slam_message = EKFSlamInput()
		ekf_slam_message.odom = average_odom
		ekf_slam_message.input = average_input
		ekf_slam_message.detections = detections_msg

		#Write to a rosbag at average time
		outbag.write('/odom', average_odom, rospy.Time(timestamp))
		outbag.write('/input', average_input, rospy.Time(timestamp))
		outbag.write('/tag_detections', detections_msg , rospy.Time(timestamp))	
		#outbag.write('/ekf_input', ekf_slam_message , rospy.Time(timestamp))	

		#print "-----------------------------------------"
		#Set bag to next time interval
		start_time = end_time
		end_time = start_time + rospy.Duration(data_averaging_time)

bag.close()

