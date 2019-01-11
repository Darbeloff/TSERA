#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
#from ik_helper import *
#from sensor_msgs.msg import Joy

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)

def ort_callback(msg):
	xyz = [0]*3
	magnitude = np.sqrt(((msg.data[0])**2) + ((msg.data[1])**2) + ((msg.data[2])**2))
	unit_vector = [msg.data[0], msg.data[1], msg.data[2]]/magnitude
	#unit vector msg
	
	#mathematica
	xyz_msg = Float32MultiArray(data = xyz)
	ort_pub.publish(xyz_msg)

def orientation():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/des_ort", Float32MultiArray, ort_callback)
	rospy.spin()

if __name__ = '__main__':
	orientation()
