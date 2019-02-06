#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, String
import numpy as np
pub = rospy.Publisher('/des_ort', Float32MultiArray, queue_size = 1)

def cb(msg):
	vector = msg.data.split()
	print vector
	X = float(vector[0])
	Y = float(vector[1])
	r = np.sqrt(X**2+Y**2)
	phi = (np.radians(55))
	r_corr = np.sin(phi)
	if r == 0:
		x = 0
		y = 0
		z_vec = 1
	elif r >= r_corr:
		scale = r_corr/r
		x = scale*X
		y = scale*Y
		z_vec = np.cos(phi)
	elif r < r_corr:
		x = X
		y = Y
		z_vec = np.cos(np.arcsin(r))
	T = [x, y, z_vec] 
	vector_list = [0,0,1,0,0,1,x,y,z_vec, 8]
	vector_msg = Float32MultiArray(data = vector_list)
	pub.publish(vector_msg)

def talker():
	rospy.init_node('test_talker')
	rospy.Subscriber("/test", String, cb)
	rospy.spin()

if __name__ == '__main__':
	talker()