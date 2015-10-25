#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import sys
bag_name = 'velocitybag.bag'
bag_name = sys.argv[1]
bag = rosbag.Bag(bag_name)
xlist = []
ylist = []
for topic, msg,t  in bag.read_messages(topics=['/odom']):
	#print msg
	#print msg.pose.pose.position.x
	#print msg.pose.pose.position.y
	xlist.append(msg.pose.pose.position.x)
	ylist.append(msg.pose.pose.position.y)
	#print '-----------------------------------------------'
plt.scatter(xlist, ylist) 
plt.show(block = "True")
bag.close()

           
