#!/usr/bin/env python

from rospy.rostime import Time
import rosbag
import math
import matplotlib.pyplot as plt
#from nav_msgs import Odometry

bag = rosbag.Bag('velocitybag.bag')
xlist = []
ylist = []
heading_list = []
#pose =
xpos = 0.0
ypos = 0.0
tpos = 0.0
t_old = Time()
for topic, msg,t  in bag.read_messages(topics=['/cmd_vel_mux/input/teleop']):
#	if(msg.angular.z != 0):
#		print msg
#		print str(xpos) + "--" + str(ypos) + "--" + str(tpos)	
	del_x = (t-t_old).to_sec() * msg.linear.x
#	del_y = (t-t_old).to_sec() * msg.linear.y
	del_t = (t-t_old).to_sec() * msg.angular.z	
#	if del_t != 0:
#		print del_t	
	tpos = tpos + del_t
	heading = tpos#= math.atan(tpos)
	heading_list.append(tpos)
	print "heading:" + str(heading)	
	#R = np.matrix([[math.cos(robot_z), -math.sin(robot_z)], [math.sin(robot_z), math.cos(robot_z)]])
	xpos = xpos + del_x* math.cos(tpos)
	ypos = ypos + del_x* math.sin(tpos)

	t_old = t
	xlist.append(xpos)
	ylist.append(ypos)
	#print '-----------------------------------------------'
plt.scatter(xlist, ylist) 
#plt.plot(xlist)
#plt.plot(ylist)
#plt.plot(heading_list)
plt.show(block = "True")
bag.close()

           
