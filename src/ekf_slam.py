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


#NOISES
alpha_1 = 0.1
alpha_2 = 0.1
alpha_3 = 0.1
alpha_4 = 0.1
alpha_5 = 0.1
alpha_6 = 0.1

#GLOBALS
num_landmarks = 0

#STATIC MATRICES
#Fx s.t. x_t = Fx*y_t
Fx_replacer = np.eye(3)

#sigma_thetha
sigma_thetha = np.matrix([[0,0,0],[0,0,0],[0,0,normal(0.0, alpha_5) + normal(0.0, alpha_6)]])

#METHODS

#Motion model
#mu_t_1 = [x,y,w,v_x,v_y,w_t].T
#u_t = [v_x,w_t].T
#NOTE: v_y is always zero
def g(mu_t_1, u_t, del_t):
	v = u_t[0]
	w = u_t[1]
	theta_t_1 = mu_t_1[2]

	temp1 = np.matrix([(-u_t[0]/u_t[1]*sin(mu_t_1[2]) + (-u_t[0]/u_t[1]*sin(mu_t_1[2] + u_t[2]*del_t)],[(u_t[0]/u_t[1]*cos(mu_t_1[2]) - (-u_t[0]/u_t[1]*cos(mu_t_1[2] + u_t[2]*del_t)],[mu_t[2]*del_t]).transpose()
	mu_t = mu_t_1 + Fx.transpose()*temp1
	return mu_t


def G(mu_t_1, u_t, del_t):	
	v = u_t[0]
	w = u_t[2]
	theta_t_1 = mu_t_1[2]
	temp2 = np.matrix([-v/w*cos(theta_t_1) + v/w*cost(thetha_t_1 + w*del_t)],[-v/w*sin(theta_t_1) + v/w*sin(thetha_t_1 + w*del_t)],[0]).transpose()
	G_t = np.eye(6) + Fx.transpose()*temp2*Fx 	
	return G_t

def get_sigma_thetha(v,w):
	return np.matrix([[0,0,0],[0,0,0],[0,0,normal(0.0, alpha_5*(v**2)) + normal(0.0, alpha_6*(w**2))]])

def sigma_u_with_noise(sigma_u, V, u):
	return V*sigma_u*V.transpose() + get_sigma_thetha(u[0],u[2])


def add_noise_to_u(v_t,w_t,theta_t_1):
	va_t = v_t + normal(0.0, alpha_1*(v**2) + alpha_2*(w**2))
	wa_t = w_t + normal(0.0, alpha_3*(v**2) + alpha_4*(w**2))
	thetaa_t_1 = theta_t_1 + normal(0.0, alpha_5*(v**2) + alpha_6*(w**2))
	return va_t, wa_t, thetaa_t_1


def ekf_localize(mu_t_1, sigma_t_1, u_t, del_t):
	mu_pred_t = g(mu_t_1, u_t, del_t)
	G_t = G(mu_t_1, u_t, del_t)
	E_t_1 = sigma_t_1
	sigma_pred_t = G_t*sigma_t_1*G_t.transpose() + sigma_u_with_noise(sigma_u, V, u_t)
	return mu_pred_t, sigma_pred_t


