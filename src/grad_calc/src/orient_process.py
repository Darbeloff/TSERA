#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
#from ik_helper import *
#from sensor_msgs.msg import Joy

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)

class positionClass():
	def __init__(self, stage_):
		self.x = 0
		self.y = 0
		self.z = 92
	def updateXYZ(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z


def ort_callback(msg):
	#Need arduino to publish current position so we can have XYZ.
	xyz = [0]*3
	magnitude = np.sqrt(((msg.data[0])**2) + ((msg.data[1])**2) + ((msg.data[2])**2))
	unit_vector = [msg.data[0], msg.data[1], msg.data[2]]/magnitude
	#unit vector msg
	
	#mathematica
	Lt = 85.3 #mm update with value from spherical joints
	s0 = 33.23 #mm
	Ltv = (2*((Lt**2)-(Lt/2)**2)**.5)/3
#Need current XYZ values to find sa,sb,sc,ha,hb,hc for NBT. 

	xyz_msg = Float32MultiArray(data = xyz)
	ort_pub.publish(xyz_msg)

def orientation():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/des_ort", Float32MultiArray, ort_callback)
	rospy.spin()

if __name__ = '__main__':
	orientation()
