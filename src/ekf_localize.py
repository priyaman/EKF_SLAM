#!/usr/bin/python
import rospy
import numpy as np
import rosbag
import sys
from tf.transformations import euler_from_quaternion
from math import sqrt
from math import sin
import math
from numpy.random import normal

#Motion model
def g(mu_t_1, u_t, del_t):
	v = u_t[0]
	w = u_t[1]
	theta = mu_t_1[2]
	temp1 = np.matrix([((-v/w)*sin(theta) + ((v/w)*sin(theta + w*del_t)), ((v/w)*cos(theta) - ((v/w)*cos(theta + w*del_t)) , w*del_t]).transpose()
	return mu_t_1 + temp1

#Linearized G
def G(mu_t_1, u_t, del_t):	
	#Extract Control Input from u_t
	v = u_t[0]
	w = u_t[1]
	#Get current rotation from pose	
	theta_t_1 = mu_t_1[2]
	
	G = np.matrix([[1.0, 0.0, -v/w*cos(theta_t_1) + v/w*cos(thetha_t_1 + w*del_t)],[0.0, 1.0, -v/w*sin(theta_t_1) + v/w*sin(thetha_t_1 + w*del_t)],[0.0, 0.0, 1.0]])
	
	#The Fx method
	#temp2 = np.matrix([-v/w*cos(theta_t_1) + v/w*cos(thetha_t_1 + w*del_t), -v/w*sin(theta_t_1) + v/w*sin(thetha_t_1 + w*del_t), 0]).transpose()
	#G_t = np.eye(3) + Fx.transpose()*temp2*Fx 	
	#G_t = np.matrix(np.eye(3)) + Fx.transpose()*temp2*Fx 	
	
	return G

#Linearize V
def V(mu_t_1, u_t, del_t):
	#Extract Control Input from u_t
	v = u_t[0]
	w = u_t[1]
	#Get current rotation from pose	
	theta = mu_t_1[2]
	
	V = np.matrix([[(-sin(theta) + sin(theta+ w*del_t))/w, (v/w**2)*(sin(theta)- sin(theta+ w*del_t)) + ((v/w)*cos(theta + w*del_t)*del_t)], [(cos(theta) - sin(theta+ w*del_t))/w, (-v/w**2)*(cos(theta)- cos(theta+ w*del_t)) + ((v/w)*sin(theta + w*del_t)*del_t)], [0 , del_t]])
	return V

#Get additive Gaussian Noise
#Shouldnt it be random sample ?
def M(u_t):
	#Extract Control Input from u_t
	v = u_t[0]
	w = u_t[1]
	M = np.matrix([[(alpha_1 * (v**2)) + (alpha_2 * (w**2)), 0.0 ], [0.0, (alpha_3 * (v**2)) + (alpha_4 * (w**2))]])
	return M

#Rotation noise
def get_sigma_thetha(u_t):
	v = u_t[0]
	w = u_t[1]
	return np.matrix([[0,0,0],[0,0,0],[0,0,normal(0.0, alpha_5*(v**2)) + normal(0.0, alpha_6*(w**2))]])




# mu_t_1 = [x_t_1, y_t_1, theta_t_1].T
def ekf_localize_known_corr(mu_t_1, sigma_t_1, u_t, z_t, c_t, m, del_t):
	#Get current rotation from pose	
	theta_t_1 = mu_t_1[2]
	#Extract Control Input from u_t
	v = u_t[0]
	w = u_t[1]

	G_t = G(mu_t_1, u_t, del_t)
	V_t = V(mu_t_1, u_t, del_t)
	M_t = M(u_t)
	sigma_theta = get_sigma_theta(u_t) 
	
	mu_pred_t = g(mu_t_1, u_t, del_t)
	sigma_pred_t = sigma_t_1 + G_t*sigma_t_1*G_t.T + V_t*M_t*V_t.T + sigma_theta
	
	


